/**
 * @file camera.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <multisense_ros/camera.h>
#include <multisense_ros/RawCamConfig.h>
#include <multisense_ros/RawCamCal.h>
#include <multisense_ros/DeviceInfo.h>
#include <multisense_ros/Histogram.h>

#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

#include <multisense_lib/MultiSenseChannel.hh>

#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <fstream>
#include <turbojpeg.h>

using namespace crl::multisense;

namespace multisense_ros {

namespace { // anonymous

//
// All of the data sources that we control here

const DataSource allImageSources = (Source_Luma_Left            |
                                    Source_Luma_Right           |
                                    Source_Luma_Rectified_Left  |
                                    Source_Luma_Rectified_Right |
                                    Source_Chroma_Left          |
                                    Source_Disparity            |
                                    Source_Disparity_Right      |
                                    Source_Disparity_Cost       |
                                    Source_Jpeg_Left);

//
// Packed size of point cloud structures

const uint32_t luma_cloud_step  = 16;
const uint32_t color_cloud_step = 16;

//
// Shims for C-style driver callbacks

void monoCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->monoCallback(header); }
void rectCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rectCallback(header); }
void depthCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->depthCallback(header); }
void pointCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->pointCloudCallback(header); }
void rawCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rawCamDataCallback(header); }
void colorCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorImageCallback(header); }
void dispCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->disparityImageCallback(header); }
void jpegCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->jpegImageCallback(header); }
void histCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->histogramCallback(header); }

//
// Check for valid range points coming out of OpenCV

bool isValidPoint(const cv::Vec3f& pt,
                  const float&     maxRange)
{
    //
    // Check both for disparities explicitly marked as invalid (where
    // OpenCV maps pt.z to MISSING_Z) and zero disparities (point
    // mapped to infinity).

    if (image_geometry::StereoCameraModel::MISSING_Z != pt[2] && std::isfinite(pt[2])) {

        //
        // Also filter on reasonable ranges

        const float mag = std::sqrt((pt[0]*pt[0])+(pt[1]*pt[1])+(pt[2]*pt[2]));

        if (mag < maxRange)
            return true;
    }

    return false;
}

//
// Publish a point cloud, using the given storage, filtering the points,
// and colorizing the cloud with all available color channels.
//
// Note that the dependencies for the point cloud will be generated in
// different threads.  This function is called each time a dependency
// becomes ready.
//
// The published frame ID for this point cloud type is tracked and
// publishing is serialized here via a mutex to prevent race conditions.

boost::mutex point_cloud_mutex;

bool publishPointCloud(int64_t                       imageFrameId,
                       int64_t                       pointsFrameId,
                       int64_t&                      cloudFrameId,
                       ros::Publisher&               pub,
                       sensor_msgs::PointCloud2&     cloud,
                       const uint32_t                width,
                       const uint32_t                height,
                       const uint32_t                timeSeconds,
                       const uint32_t                timeMicroSeconds,
                       const uint32_t                cloudStep,
                       const std::vector<cv::Vec3f>& points,
                       const uint8_t*                imageP,
                       const uint32_t                colorChannels,
                       const float                   maxRange,
                       bool                          writeColorPacked,
                       bool                          organized)
{
    boost::mutex::scoped_lock lock(point_cloud_mutex);

    if (0            == pub.getNumSubscribers()  ||
        imageFrameId != pointsFrameId            ||
        cloudFrameId >= imageFrameId) {
        return false;
    }

    cloudFrameId = imageFrameId;
    const uint32_t imageSize = height * width;

    if (points.size() != imageSize)
        return false;

    cloud.data.resize(imageSize * cloudStep);

    uint8_t       *cloudP      = reinterpret_cast<uint8_t*>(&cloud.data[0]);
    const uint32_t pointSize   = 3 * sizeof(float); // x, y, z
    uint32_t       validPoints = 0;

    cv::Vec3f nanPoint(std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::quiet_NaN(),
                       std::numeric_limits<float>::quiet_NaN());


    for(uint32_t i=0; i<height; ++i)
        for(uint32_t j=0; j<width; ++j) {

            const uint32_t index = i * width + j;

            const uint32_t* pointP = reinterpret_cast<const uint32_t*>(&points[index]);
            uint32_t* targetCloudP = reinterpret_cast<uint32_t*>(cloudP);


            //
            // When creating an organized pointcloud replace invalid points
            // with NaN points

            if (false == isValidPoint(points[index], maxRange))
            {
                if (organized)
                {
                    pointP = reinterpret_cast<const uint32_t*>(&nanPoint[0]);
                }
                else
                {
                    continue;
                }
            }


            //
            // Directly copy points to eliminate memcpy

            targetCloudP[0] = pointP[0];
            targetCloudP[1] = pointP[1];
            targetCloudP[2] = pointP[2];


            const uint8_t *sourceColorP = &(imageP[colorChannels * index]);
            uint8_t       *cloudColorP  = (cloudP + pointSize);

            //
            // Write the poincloud packed if specified or the color image
            // is BGR. Copying is optimized to eliminate memcpy operations
            // increasing overall speed

            if (writeColorPacked || colorChannels > 2)
            {
                switch(colorChannels)
                {
                    case 4:
                        cloudColorP[3] = sourceColorP[3];
                    case 3:
                        cloudColorP[2] = sourceColorP[2];
                    case 2:
                        cloudColorP[1] = sourceColorP[1];
                    case 1:
                        cloudColorP[0] = sourceColorP[0];
                }

            } else {
                union
                {
                    uint32_t value;
                    char bytes[sizeof(uint32_t)];
                } color;

                color.value = 0;

                //
                // We only need to copy 2 values since this case only
                // applies for images with color channels of 1 or 2 bytes

                color.bytes[0] = sourceColorP[0];
                color.bytes[1] = sourceColorP[1] & ((colorChannels > 1) * 255);

                float* floatCloudColorP = reinterpret_cast<float*>(cloudColorP);
                floatCloudColorP[0] = static_cast<float>(color.value);
            }

            cloudP += cloudStep;
            validPoints ++;
        }

    if (!organized) {
        cloud.row_step     = validPoints * cloudStep;
        cloud.width        = validPoints;
        cloud.height       = 1;
    } else {
        cloud.width = width;
        cloud.height = height;
        cloud.row_step = width * cloudStep;
    }

    cloud.header.stamp = ros::Time(timeSeconds, 1000 * timeMicroSeconds);
    cloud.data.resize(validPoints * cloudStep);
    pub.publish(cloud);

    return true;
}

bool savePgm(const std::string& fileName,
             uint32_t           width,
             uint32_t           height,
             uint32_t           bitsPerPixel,
             const void        *dataP)
{
    std::ofstream outputStream(fileName.c_str(), std::ios::binary | std::ios::out);

    if (false == outputStream.good()) {
        fprintf(stderr, "failed to open \"%s\"\n", fileName.c_str());
        return false;
    }

    const uint32_t imageSize = height * width;

    switch(bitsPerPixel) {
    case 8:
    {

        outputStream << "P5\n"
                     << width << " " << height << "\n"
                     << 0xFF << "\n";

        outputStream.write(reinterpret_cast<const char*>(dataP), imageSize);

        break;
    }
    case 16:
    {
        outputStream << "P5\n"
                     << width << " " << height << "\n"
                     << 0xFFFF << "\n";

        const uint16_t *imageP = reinterpret_cast<const uint16_t*>(dataP);

        for (uint32_t i=0; i<imageSize; ++i) {
            uint16_t o = htons(imageP[i]);
            outputStream.write(reinterpret_cast<const char*>(&o), sizeof(uint16_t));
        }

        break;
    }
    }

    outputStream.close();
    return true;
}

}; // anonymous

Camera::Camera(Channel* driver,
               const std::string& tf_prefix) :
    driver_(driver),
    device_nh_(""),
    left_nh_(device_nh_, "left"),
    right_nh_(device_nh_, "right"),
    left_mono_transport_(left_nh_),
    right_mono_transport_(right_nh_),
    left_rect_transport_(left_nh_),
    right_rect_transport_(right_nh_),
    left_rgb_transport_(left_nh_),
    left_rgb_rect_transport_(left_nh_),
    depth_transport_(device_nh_),
    ni_depth_transport_(device_nh_),
    disparity_left_transport_(left_nh_),
    disparity_right_transport_(right_nh_),
    disparity_cost_transport_(left_nh_),
    left_mono_cam_info_(),
    right_mono_cam_info_(),
    left_rect_cam_info_(),
    right_rect_cam_info_(),
    left_rgb_rect_cam_info_(),
    left_disp_cam_info_(),
    right_disp_cam_info_(),
    left_cost_cam_info_(),
    left_rgb_cam_info_(),
    depth_cam_info_(),
    left_mono_cam_pub_(),
    right_mono_cam_pub_(),
    left_rect_cam_pub_(),
    right_rect_cam_pub_(),
    depth_cam_pub_(),
    ni_depth_cam_pub_(),
    left_rgb_cam_pub_(),
    left_rgb_rect_cam_pub_(),
    left_mono_cam_info_pub_(),
    right_mono_cam_info_pub_(),
    left_rect_cam_info_pub_(),
    right_rect_cam_info_pub_(),
    left_disp_cam_info_pub_(),
    right_disp_cam_info_pub_(),
    left_cost_cam_info_pub_(),
    left_rgb_cam_info_pub_(),
    left_rgb_rect_cam_info_pub_(),
    depth_cam_info_pub_(),
    luma_point_cloud_pub_(),
    color_point_cloud_pub_(),
    luma_organized_point_cloud_pub_(),
    color_organized_point_cloud_pub_(),
    left_disparity_pub_(),
    right_disparity_pub_(),
    left_disparity_cost_pub_(),
    left_stereo_disparity_pub_(),
    right_stereo_disparity_pub_(),
    raw_cam_data_pub_(),
    raw_cam_config_pub_(),
    raw_cam_cal_pub_(),
    device_info_pub_(),
    histogram_pub_(),
    left_mono_image_(),
    right_mono_image_(),
    left_rect_image_(),
    right_rect_image_(),
    depth_image_(),
    luma_point_cloud_(),
    color_point_cloud_(),
    luma_organized_point_cloud_(),
    color_organized_point_cloud_(),
    left_luma_image_(),
    left_rgb_image_(),
    left_rgb_rect_image_(),
    left_disparity_image_(),
    left_disparity_cost_image_(),
    right_disparity_image_(),
    left_stereo_disparity_(),
    right_stereo_disparity_(),
    got_raw_cam_left_(false),
    got_left_luma_(false),
    left_luma_frame_id_(0),
    left_rect_frame_id_(0),
    left_rgb_rect_frame_id_(-1),
    luma_point_cloud_frame_id_(-1),
    luma_organized_point_cloud_frame_id_(-1),
    color_point_cloud_frame_id_(-1),
    color_organized_point_cloud_frame_id_(-1),
    raw_cam_data_(),
    version_info_(),
    device_info_(),
    image_config_(),
    image_calibration_(),
    cal_lock_(),
    calibration_map_left_1_(),
    calibration_map_left_2_(),
    frame_id_left_(),
    frame_id_right_(),
    disparity_buff_(),
    points_buff_(),
    points_buff_frame_id_(-1),
    q_matrix_(4, 4, 0.0),
    pc_max_range_(15.0f),
    pc_color_frame_sync_(true),
    disparities_(0),
    stream_lock_(),
    stream_map_(),
    last_frame_id_(-1),
    luma_color_depth_(1),
    write_pc_color_packed_(false),
    border_clip_type_(0),
    border_clip_value_(0.0),
    border_clip_lock_()
{
    //
    // Query device and version information from sensor

    Status status = driver_->getVersionInfo(version_info_);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query version info: %s",
                  Channel::statusString(status));
        return;
    }
    status = driver_->getDeviceInfo(device_info_);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    //
    // Set frame ID

    frame_id_left_  = tf_prefix + "/left_camera_optical_frame";
    frame_id_right_ = tf_prefix + "/right_camera_optical_frame";
    ROS_INFO("camera frame id: %s", frame_id_left_.c_str());

    //
    // Topics published for all device types

    ros::NodeHandle calibration_nh(device_nh_, "calibration");
    device_info_pub_    = calibration_nh.advertise<multisense_ros::DeviceInfo>("device_info", 1, true);
    raw_cam_cal_pub_    = calibration_nh.advertise<multisense_ros::RawCamCal>("raw_cam_cal", 1, true);
    raw_cam_config_pub_ = calibration_nh.advertise<multisense_ros::RawCamConfig>("raw_cam_config", 1, true);
    histogram_pub_      = device_nh_.advertise<multisense_ros::Histogram>("histogram", 5);

    //
    // Change the way the luma pointcloud is published for ST21 sensors

    if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 == device_info_.hardwareRevision) {

        //
        // Luma images are 16 bit so when copying to the point cloud
        // structure copy 2 bytes

        luma_color_depth_ = 2;
    }

    //
    // Create topic publishers (TODO: color topics should not be advertised if the device can't support it)

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        left_mono_cam_pub_  = left_mono_transport_.advertise("image_mono", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));

        left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));

        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));

        left_mono_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_mono/camera_info", 1, true);
        left_rgb_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_color/camera_info", 1, true);
        left_rgb_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_rect_color/camera_info", 1, true);


    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_M == device_info_.hardwareRevision) {

        // monocular variation

        left_mono_cam_pub_  = left_mono_transport_.advertise("image_mono", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        left_rect_cam_pub_  = left_rect_transport_.advertiseCamera("image_rect", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left));
        left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                                 boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                 boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));

        left_mono_cam_info_pub_     = left_nh_.advertise<sensor_msgs::CameraInfo>("image_mono/camera_info", 1, true);
        left_rect_cam_info_pub_     = left_nh_.advertise<sensor_msgs::CameraInfo>("image_rect/camera_info", 1, true);
        left_rgb_cam_info_pub_      = left_nh_.advertise<sensor_msgs::CameraInfo>("image_color/camera_info", 1, true);
        left_rgb_rect_cam_info_pub_ = left_nh_.advertise<sensor_msgs::CameraInfo>("image_rect_color/camera_info", 1, true);

    } else {  // all other MultiSense-S* variations


        left_mono_cam_pub_  = left_mono_transport_.advertise("image_mono", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        right_mono_cam_pub_ = right_mono_transport_.advertise("image_mono", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Right),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Right));
        left_rect_cam_pub_  = left_rect_transport_.advertiseCamera("image_rect", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left));
        right_rect_cam_pub_ = right_rect_transport_.advertiseCamera("image_rect", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Right),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Right));
        depth_cam_pub_      = depth_transport_.advertise("depth", 5,
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Disparity));
        ni_depth_cam_pub_   = ni_depth_transport_.advertise("openni_depth", 5,
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Disparity));

        if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 != device_info_.hardwareRevision) {

            left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                                  boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                  boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
            left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                                  boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                  boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
            color_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("image_points2_color", 5,
                                  boost::bind(&Camera::connectStream, this,
                                  Source_Disparity | Source_Luma_Left | Source_Chroma_Left),
                                  boost::bind(&Camera::disconnectStream, this,
                                  Source_Disparity | Source_Luma_Left | Source_Chroma_Left));
            color_organized_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("organized_image_points2_color", 5,
                                  boost::bind(&Camera::connectStream, this,
                                  Source_Disparity | Source_Luma_Left | Source_Chroma_Left),
                                  boost::bind(&Camera::disconnectStream, this,
                                  Source_Disparity | Source_Luma_Left | Source_Chroma_Left));

            left_rgb_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_color/camera_info", 1, true);
            left_rgb_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_rect_color/camera_info", 1, true);

        }

        luma_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("image_points2", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        luma_organized_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("organized_image_points2", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        raw_cam_data_pub_   = calibration_nh.advertise<multisense_ros::RawCamData>("raw_cam_data", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        left_disparity_pub_ = disparity_left_transport_.advertise("disparity", 5,
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Disparity));

        left_stereo_disparity_pub_ = left_nh_.advertise<stereo_msgs::DisparityImage>("disparity_image", 5,
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Disparity));

        if (version_info_.sensorFirmwareVersion >= 0x0300) {

            right_disparity_pub_ = disparity_right_transport_.advertise("disparity", 5,
                                   boost::bind(&Camera::connectStream, this, Source_Disparity_Right),
                                   boost::bind(&Camera::disconnectStream, this, Source_Disparity_Right));

            right_stereo_disparity_pub_ = right_nh_.advertise<stereo_msgs::DisparityImage>("disparity_image", 5,
                                  boost::bind(&Camera::connectStream, this, Source_Disparity_Right),
                                  boost::bind(&Camera::disconnectStream, this, Source_Disparity_Right));

            left_disparity_cost_pub_ = disparity_cost_transport_.advertise("cost", 5,
                                   boost::bind(&Camera::connectStream, this, Source_Disparity_Cost),
                                   boost::bind(&Camera::disconnectStream, this, Source_Disparity_Cost));

            right_disp_cam_info_pub_  = right_nh_.advertise<sensor_msgs::CameraInfo>("disparity/camera_info", 1, true);
            left_cost_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("cost/camera_info", 1, true);
        }

        //
        // Camera info topic publishers
        left_mono_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_mono/camera_info", 1, true);
        right_mono_cam_info_pub_ = right_nh_.advertise<sensor_msgs::CameraInfo>("image_mono/camera_info", 1, true);
        left_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("image_rect/camera_info", 1, true);
        right_rect_cam_info_pub_ = right_nh_.advertise<sensor_msgs::CameraInfo>("image_rect/camera_info", 1, true);
        left_disp_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>("disparity/camera_info", 1, true);
        depth_cam_info_pub_ = device_nh_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1, true);
    }


    //
    // All image streams off

    stop();

    //
    // Publish device info

    multisense_ros::DeviceInfo msg;

    msg.deviceName     = device_info_.name;
    msg.buildDate      = device_info_.buildDate;
    msg.serialNumber   = device_info_.serialNumber;
    msg.deviceRevision = device_info_.hardwareRevision;

    msg.numberOfPcbs = device_info_.pcbs.size();
    std::vector<system::PcbInfo>::const_iterator it = device_info_.pcbs.begin();
    for(; it != device_info_.pcbs.end(); ++it) {
        msg.pcbSerialNumbers.push_back((*it).revision);
        msg.pcbNames.push_back((*it).name);
    }

    msg.imagerName              = device_info_.imagerName;
    msg.imagerType              = device_info_.imagerType;
    msg.imagerWidth             = device_info_.imagerWidth;
    msg.imagerHeight            = device_info_.imagerHeight;

    msg.lensName                = device_info_.lensName;
    msg.lensType                = device_info_.lensType;
    msg.nominalBaseline         = device_info_.nominalBaseline;
    msg.nominalFocalLength      = device_info_.nominalFocalLength;
    msg.nominalRelativeAperture = device_info_.nominalRelativeAperture;

    msg.lightingType            = device_info_.lightingType;
    msg.numberOfLights          = device_info_.numberOfLights;

    msg.laserName               = device_info_.laserName;
    msg.laserType               = device_info_.laserType;

    msg.motorName               = device_info_.motorName;
    msg.motorType               = device_info_.motorType;
    msg.motorGearReduction      = device_info_.motorGearReduction;

    msg.apiBuildDate            = version_info_.apiBuildDate;
    msg.apiVersion              = version_info_.apiVersion;
    msg.firmwareBuildDate       = version_info_.sensorFirmwareBuildDate;
    msg.firmwareVersion         = version_info_.sensorFirmwareVersion;
    msg.bitstreamVersion        = version_info_.sensorHardwareVersion;
    msg.bitstreamMagic          = version_info_.sensorHardwareMagic;
    msg.fpgaDna                 = version_info_.sensorFpgaDna;

    device_info_pub_.publish(msg);

    //
    // Publish image calibration

    status = driver_->getImageCalibration(image_calibration_);
    if (Status_Ok != status)
        ROS_ERROR("Camera: failed to query image calibration: %s",
                  Channel::statusString(status));
    else {

        multisense_ros::RawCamCal cal;
        const float              *cP;

        cP = reinterpret_cast<const float *>(&(image_calibration_.left.M[0][0]));
        for(uint32_t i=0; i<9; i++) cal.left_M[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.left.D[0]));
        for(uint32_t i=0; i<8; i++) cal.left_D[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.left.R[0][0]));
        for(uint32_t i=0; i<9; i++) cal.left_R[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.left.P[0][0]));
        for(uint32_t i=0; i<12; i++) cal.left_P[i] = cP[i];

        cP = reinterpret_cast<const float *>(&(image_calibration_.right.M[0][0]));
        for(uint32_t i=0; i<9; i++) cal.right_M[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.right.D[0]));
        for(uint32_t i=0; i<8; i++) cal.right_D[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.right.R[0][0]));
        for(uint32_t i=0; i<9; i++) cal.right_R[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration_.right.P[0][0]));
        for(uint32_t i=0; i<12; i++) cal.right_P[i] = cP[i];

        raw_cam_cal_pub_.publish(cal);
    }

    //
    // Get current sensor configuration

    q_matrix_(0,0) = q_matrix_(1,1) = 1.0;
    queryConfig();

    //
    // Initialze point cloud data structures

    luma_point_cloud_.is_bigendian    = (htonl(1) == 1);
    luma_point_cloud_.is_dense        = true;
    luma_point_cloud_.point_step      = luma_cloud_step;
    luma_point_cloud_.height          = 1;
    luma_point_cloud_.header.frame_id = frame_id_left_;
    luma_point_cloud_.fields.resize(4);
    luma_point_cloud_.fields[0].name     = "x";
    luma_point_cloud_.fields[0].offset   = 0;
    luma_point_cloud_.fields[0].count    = 1;
    luma_point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    luma_point_cloud_.fields[1].name     = "y";
    luma_point_cloud_.fields[1].offset   = 4;
    luma_point_cloud_.fields[1].count    = 1;
    luma_point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    luma_point_cloud_.fields[2].name     = "z";
    luma_point_cloud_.fields[2].offset   = 8;
    luma_point_cloud_.fields[2].count    = 1;
    luma_point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    luma_point_cloud_.fields[3].name     = "luminance";
    luma_point_cloud_.fields[3].offset   = 12;
    luma_point_cloud_.fields[3].count    = 1;
    luma_point_cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    color_point_cloud_.is_bigendian    = (htonl(1) == 1);
    color_point_cloud_.is_dense        = true;
    color_point_cloud_.point_step      = color_cloud_step;
    color_point_cloud_.height          = 1;
    color_point_cloud_.header.frame_id = frame_id_left_;
    color_point_cloud_.fields.resize(4);
    color_point_cloud_.fields[0].name     = "x";
    color_point_cloud_.fields[0].offset   = 0;
    color_point_cloud_.fields[0].count    = 1;
    color_point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    color_point_cloud_.fields[1].name     = "y";
    color_point_cloud_.fields[1].offset   = 4;
    color_point_cloud_.fields[1].count    = 1;
    color_point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    color_point_cloud_.fields[2].name     = "z";
    color_point_cloud_.fields[2].offset   = 8;
    color_point_cloud_.fields[2].count    = 1;
    color_point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    color_point_cloud_.fields[3].name     = "rgb";
    color_point_cloud_.fields[3].offset   = 12;
    color_point_cloud_.fields[3].count    = 1;
    color_point_cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    luma_organized_point_cloud_.is_bigendian    = (htonl(1) == 1);
    luma_organized_point_cloud_.is_dense        = false;
    luma_organized_point_cloud_.point_step      = luma_cloud_step;
    luma_organized_point_cloud_.header.frame_id = frame_id_left_;
    luma_organized_point_cloud_.fields.resize(4);
    luma_organized_point_cloud_.fields[0].name     = "x";
    luma_organized_point_cloud_.fields[0].offset   = 0;
    luma_organized_point_cloud_.fields[0].count    = 1;
    luma_organized_point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    luma_organized_point_cloud_.fields[1].name     = "y";
    luma_organized_point_cloud_.fields[1].offset   = 4;
    luma_organized_point_cloud_.fields[1].count    = 1;
    luma_organized_point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    luma_organized_point_cloud_.fields[2].name     = "z";
    luma_organized_point_cloud_.fields[2].offset   = 8;
    luma_organized_point_cloud_.fields[2].count    = 1;
    luma_organized_point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    luma_organized_point_cloud_.fields[3].name     = "luminance";
    luma_organized_point_cloud_.fields[3].offset   = 12;
    luma_organized_point_cloud_.fields[3].count    = 1;
    luma_organized_point_cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    color_organized_point_cloud_.is_bigendian    = (htonl(1) == 1);
    color_organized_point_cloud_.is_dense        = false;
    color_organized_point_cloud_.point_step      = color_cloud_step;
    color_organized_point_cloud_.header.frame_id = frame_id_left_;
    color_organized_point_cloud_.fields.resize(4);
    color_organized_point_cloud_.fields[0].name     = "x";
    color_organized_point_cloud_.fields[0].offset   = 0;
    color_organized_point_cloud_.fields[0].count    = 1;
    color_organized_point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    color_organized_point_cloud_.fields[1].name     = "y";
    color_organized_point_cloud_.fields[1].offset   = 4;
    color_organized_point_cloud_.fields[1].count    = 1;
    color_organized_point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    color_organized_point_cloud_.fields[2].name     = "z";
    color_organized_point_cloud_.fields[2].offset   = 8;
    color_organized_point_cloud_.fields[2].count    = 1;
    color_organized_point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    color_organized_point_cloud_.fields[3].name     = "rgb";
    color_organized_point_cloud_.fields[3].offset   = 12;
    color_organized_point_cloud_.fields[3].count    = 1;
    color_organized_point_cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    //
    // Add driver-level callbacks.
    //
    //    -Driver creates individual background thread for each callback.
    //    -Images are queued (depth=5) per callback, with oldest silently dropped if not keeping up.
    //    -All images presented are backed by a referenced buffer system (no copying of image data is done.)

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        driver_->addIsolatedCallback(monoCB, Source_Luma_Left, this);
        driver_->addIsolatedCallback(jpegCB, Source_Jpeg_Left, this);

    } else {

        driver_->addIsolatedCallback(monoCB,  Source_Luma_Left | Source_Luma_Right, this);
        driver_->addIsolatedCallback(rectCB,  Source_Luma_Rectified_Left | Source_Luma_Rectified_Right, this);
        driver_->addIsolatedCallback(depthCB, Source_Disparity, this);
        driver_->addIsolatedCallback(pointCB, Source_Disparity, this);
        driver_->addIsolatedCallback(rawCB,   Source_Disparity | Source_Luma_Rectified_Left, this);
        driver_->addIsolatedCallback(colorCB, Source_Luma_Left | Source_Chroma_Left, this);
        driver_->addIsolatedCallback(dispCB,  Source_Disparity | Source_Disparity_Right | Source_Disparity_Cost, this);
    }

    //
    // A common callback to publish histograms

    driver_->addIsolatedCallback(histCB, allImageSources, this);


    //
    // Disable color point cloud strict frame syncing, if desired

    const char *pcColorFrameSyncEnvStringP = getenv("MULTISENSE_ROS_PC_COLOR_FRAME_SYNC_OFF");
    if (NULL != pcColorFrameSyncEnvStringP) {
        pc_color_frame_sync_ = false;
        ROS_INFO("color point cloud frame sync is disabled");
    }
}

Camera::~Camera()
{
    stop();

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        driver_->removeIsolatedCallback(monoCB);
        driver_->removeIsolatedCallback(jpegCB);

    } else {

        driver_->removeIsolatedCallback(monoCB);
        driver_->removeIsolatedCallback(rectCB);
        driver_->removeIsolatedCallback(depthCB);
        driver_->removeIsolatedCallback(pointCB);
        driver_->removeIsolatedCallback(rawCB);
        driver_->removeIsolatedCallback(colorCB);
        driver_->removeIsolatedCallback(dispCB);
    }
}

void Camera::histogramCallback(const image::Header& header)
{
    if (last_frame_id_ >= header.frameId)
        return;

    last_frame_id_ = header.frameId;

    if (histogram_pub_.getNumSubscribers() > 0) {
        multisense_ros::Histogram rh;
        image::Histogram          mh;

        Status status = driver_->getImageHistogram(header.frameId, mh);
        if (Status_Ok == status) {
            rh.frame_count = header.frameId;
            rh.time_stamp  = ros::Time(header.timeSeconds,
                                       1000 * header.timeMicroSeconds);
            rh.width  = header.width;
            rh.height = header.height;
            switch(header.source) {
            case Source_Chroma_Left:
            case Source_Chroma_Right:
                rh.width  *= 2;
                rh.height *= 2;
            }

            rh.exposure_time = header.exposure;
            rh.gain          = header.gain;
            rh.fps           = header.framesPerSecond;
            rh.channels      = mh.channels;
            rh.bins          = mh.bins;
            rh.data          = mh.data;
            histogram_pub_.publish(rh);
        }
    }
}

void Camera::jpegImageCallback(const image::Header& header)
{
    if (Source_Jpeg_Left != header.source)
        return;

    const uint32_t height    = header.height;
    const uint32_t width     = header.width;
    const uint32_t rgbLength = height * width * 3;

    left_rgb_image_.header.frame_id = frame_id_left_;
    left_rgb_image_.height          = height;
    left_rgb_image_.width           = width;
    left_rgb_image_.encoding        = "rgb8";
    left_rgb_image_.is_bigendian    = false;
    left_rgb_image_.step            = 3 * width;
    left_rgb_image_.header.stamp    = ros::Time(header.timeSeconds,
                                                1000 * header.timeMicroSeconds);

    left_rgb_image_.data.resize(rgbLength);

    tjhandle jpegDecompressor = tjInitDecompress();
    tjDecompress2(jpegDecompressor,
                  reinterpret_cast<unsigned char*>(const_cast<void*>(header.imageDataP)),
                  header.imageLength,
                  &(left_rgb_image_.data[0]),
                  width, 0/*pitch*/, height, TJPF_RGB, 0);
    tjDestroy(jpegDecompressor);

    left_rgb_cam_pub_.publish(left_rgb_image_);
    left_rgb_cam_info_.header = left_rgb_image_.header;
    left_rgb_cam_info_pub_.publish(left_rgb_cam_info_);

    if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0) {
        boost::mutex::scoped_lock lock(cal_lock_);

        if (width  != image_config_.width() ||
            height != image_config_.height()){
            //ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
            //width, height, image_config_.width(), image_config_.height());
            cal_lock_.unlock();
            queryConfig();
        }
        else if (calibration_map_left_1_.empty() || calibration_map_left_2_.empty())
            ROS_ERROR("Camera: undistort maps not initialized");
        else {
            left_rgb_rect_image_.data.resize(rgbLength);

            const cv::Mat rgb_image(height, width, CV_8UC3, &(left_rgb_image_.data[0]));
            cv::Mat rect_rgb_image(height, width, CV_8UC3, &(left_rgb_rect_image_.data[0]));

            cv::remap(rgb_image, rect_rgb_image, calibration_map_left_1_, calibration_map_left_2_, cv::INTER_CUBIC);

            left_rgb_rect_image_.header.frame_id = frame_id_left_;
            left_rgb_rect_image_.header.stamp    = ros::Time(header.timeSeconds,
                                                             1000 * header.timeMicroSeconds);
            left_rgb_rect_image_.height          = height;
            left_rgb_rect_image_.width           = width;
            left_rgb_rect_image_.encoding        = "rgb8";
            left_rgb_rect_image_.is_bigendian    = false;
            left_rgb_rect_image_.step            = 3 * width;
            left_rgb_rect_cam_info_.header       = left_rgb_rect_image_.header;
            left_rgb_rect_frame_id_              = header.frameId;
            left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rgb_rect_cam_info_);
            left_rgb_rect_cam_info_pub_.publish(left_rgb_rect_cam_info_);
        }
    }
}

void Camera::disparityImageCallback(const image::Header& header)
{
    if (!((Source_Disparity == header.source &&
           left_disparity_pub_.getNumSubscribers() > 0) ||
          (Source_Disparity_Right == header.source &&
           right_disparity_pub_.getNumSubscribers() > 0) ||
          (Source_Disparity_Cost == header.source &&
           left_disparity_cost_pub_.getNumSubscribers() > 0) ||
          (Source_Disparity == header.source &&
           left_stereo_disparity_pub_.getNumSubscribers() > 0) ||
          (Source_Disparity_Right == header.source &&
           right_stereo_disparity_pub_.getNumSubscribers() > 0) ))
        return;

    const uint32_t imageSize = (header.width * header.height * header.bitsPerPixel) / 8;

    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Disparity:
    case Source_Disparity_Right:
    {
        sensor_msgs::Image         *imageP   = NULL;
        sensor_msgs::CameraInfo    *camInfoP = NULL;
        image_transport::Publisher *pubP     = NULL;
        ros::Publisher *camInfoPubP          = NULL;
        ros::Publisher *stereoDisparityPubP  = NULL;
        stereo_msgs::DisparityImage *stereoDisparityImageP = NULL;


        if (Source_Disparity == header.source) {
            pubP                    = &left_disparity_pub_;
            imageP                  = &left_disparity_image_;
            imageP->header.frame_id = frame_id_left_;
            camInfoP                = &left_disp_cam_info_;
            camInfoPubP             = &left_disp_cam_info_pub_;
            stereoDisparityPubP     = &left_stereo_disparity_pub_;
            stereoDisparityImageP   = &left_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_left_;
        } else {
            pubP                    = &right_disparity_pub_;
            imageP                  = &right_disparity_image_;
            imageP->header.frame_id = frame_id_right_;
            camInfoP                = &right_disp_cam_info_;
            camInfoPubP             = &right_disp_cam_info_pub_;
            stereoDisparityPubP     = &right_stereo_disparity_pub_;
            stereoDisparityImageP   = &right_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_right_;
        }



        if (pubP->getNumSubscribers() > 0)
        {
            imageP->data.resize(imageSize);
            memcpy(&imageP->data[0], header.imageDataP, imageSize);

            imageP->header.stamp    = t;
            imageP->height          = header.height;
            imageP->width           = header.width;
            imageP->is_bigendian    = false;

            switch(header.bitsPerPixel) {
                case 8:
                    imageP->encoding = sensor_msgs::image_encodings::MONO8;
                    imageP->step     = header.width;
                    break;
                case 16:
                    imageP->encoding = sensor_msgs::image_encodings::MONO16;
                    imageP->step     = header.width * 2;
                    break;
            }

            pubP->publish(*imageP);
        }

        if (stereoDisparityPubP->getNumSubscribers() > 0)
        {
            //
            // If our current image resolution is using non-square pixels, i.e.
            // fx != fy then warn the user. This support is lacking in
            // stereo_msgs::DisparityImage and stereo_image_proc

            if (right_rect_cam_info_.P[0] != right_rect_cam_info_.P[5])
            {
                std::stringstream warning;
                warning << "Current camera configuration has non-square pixels (fx != fy).";
                warning << "The stereo_msgs/DisparityImage does not account for";
                warning << " this. Be careful when reprojecting to a pointcloud.";
                ROS_WARN("%s", warning.str().c_str());
            }

            //
            // Our final floating point image will be serialized into uint8_t
            // meaning we need to allocate 4 bytes per pixel

            uint32_t floatingPointImageSize = header.width * header.height * 4;
            stereoDisparityImageP->image.data.resize(floatingPointImageSize);

            stereoDisparityImageP->header.stamp = t;

            stereoDisparityImageP->image.height = header.height;
            stereoDisparityImageP->image.width = header.width;
            stereoDisparityImageP->image.is_bigendian = false;
            stereoDisparityImageP->image.header.stamp = t;
            stereoDisparityImageP->image.header.frame_id = stereoDisparityImageP->header.frame_id;
            stereoDisparityImageP->image.encoding = "32FC1";
            stereoDisparityImageP->image.step = 4 * header.width;


            //
            // Fx is the same for both the right and left cameras

            stereoDisparityImageP->f = right_rect_cam_info_.P[0];

            //
            // Our Tx is negative. The DisparityImage message expects Tx to be
            // positive

            stereoDisparityImageP->T = fabs(right_rect_cam_info_.P[3] /
                                       right_rect_cam_info_.P[0]);
            stereoDisparityImageP->min_disparity = 0;
            stereoDisparityImageP->max_disparity = disparities_;
            stereoDisparityImageP->delta_d = 1./16.;

            //
            // The stereo_msgs::DisparityImage message expects the disparity
            // image to be floating point. We will use OpenCV to perform our
            // element-wise division


            cv::Mat_<uint16_t> tmpImage(header.height,
                                        header.width,
                                        reinterpret_cast<uint16_t*>(
                                        const_cast<void*>(header.imageDataP)));

            //
            // We will copy our data directly into our output message

            cv::Mat_<float> floatingPointImage(header.height,
                                               header.width,
                                               reinterpret_cast<float*>(&stereoDisparityImageP->image.data[0]));

            //
            // Convert our disparity to floating point by dividing by 16 and
            // copy the result to the output message

            floatingPointImage = tmpImage / 16.0;

            stereoDisparityPubP->publish(*stereoDisparityImageP);
        }

        camInfoP->header = imageP->header;
        camInfoP->header.stamp = t;
        camInfoPubP->publish(*camInfoP);

        break;
    }
    case Source_Disparity_Cost:

        left_disparity_cost_image_.data.resize(imageSize);
        memcpy(&left_disparity_cost_image_.data[0], header.imageDataP, imageSize);

        left_disparity_cost_image_.header.frame_id = frame_id_left_;
        left_disparity_cost_image_.header.stamp    = t;
        left_disparity_cost_image_.height          = header.height;
        left_disparity_cost_image_.width           = header.width;

        left_disparity_cost_image_.encoding        = sensor_msgs::image_encodings::MONO8;
        left_disparity_cost_image_.is_bigendian    = false;
        left_disparity_cost_image_.step            = header.width;

        left_disparity_cost_pub_.publish(left_disparity_cost_image_);

        left_cost_cam_info_.header = left_disparity_cost_image_.header;
        left_cost_cam_info_pub_.publish(left_cost_cam_info_);

        break;
    }
}

void Camera::monoCallback(const image::Header& header)
{
    if (Source_Luma_Left  != header.source &&
        Source_Luma_Right != header.source) {

        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }

    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Left:

        left_mono_image_.data.resize(header.imageLength);
        memcpy(&left_mono_image_.data[0], header.imageDataP, header.imageLength);

        left_mono_image_.header.frame_id = frame_id_left_;
        left_mono_image_.header.stamp    = t;
        left_mono_image_.height          = header.height;
        left_mono_image_.width           = header.width;

        switch(header.bitsPerPixel) {
            case 8:
                left_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                left_mono_image_.step     = header.width;
                break;
            case 16:
                left_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                left_mono_image_.step     = header.width * 2;
                break;
        }

        left_mono_image_.is_bigendian    = false;

        left_mono_cam_pub_.publish(left_mono_image_);

        //
        // Publish a specific camera info message for the left mono image
        left_mono_cam_info_.header= left_mono_image_.header;
        left_mono_cam_info_pub_.publish(left_mono_cam_info_);

        break;
    case Source_Luma_Right:

        right_mono_image_.data.resize(header.imageLength);
        memcpy(&right_mono_image_.data[0], header.imageDataP, header.imageLength);

        right_mono_image_.header.frame_id = frame_id_right_;
        right_mono_image_.header.stamp    = t;
        right_mono_image_.height          = header.height;
        right_mono_image_.width           = header.width;

        switch(header.bitsPerPixel) {
            case 8:
                right_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                right_mono_image_.step     = header.width;
                break;
            case 16:
                right_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                right_mono_image_.step     = header.width * 2;
                break;
        }
        right_mono_image_.is_bigendian    = false;

        right_mono_cam_pub_.publish(right_mono_image_);

        //
        // Publish a specific camera info message for the left mono image
        right_mono_cam_info_.header= right_mono_image_.header;
        right_mono_cam_info_pub_.publish(right_mono_cam_info_);

        break;
    }
}

void Camera::rectCallback(const image::Header& header)
{
    if (Source_Luma_Rectified_Left  != header.source &&
        Source_Luma_Rectified_Right != header.source) {

        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }


    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Rectified_Left:

        left_rect_image_.data.resize(header.imageLength);
        memcpy(&left_rect_image_.data[0], header.imageDataP, header.imageLength);

        left_rect_image_.header.frame_id = frame_id_left_;
        left_rect_image_.header.stamp    = t;
        left_rect_image_.height          = header.height;
        left_rect_image_.width           = header.width;

        left_rect_frame_id_              = header.frameId;


        switch(header.bitsPerPixel) {
            case 8:
                left_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                left_rect_image_.step     = header.width;

                break;
            case 16:
                left_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                left_rect_image_.step     = header.width * 2;

                break;
        }

        left_rect_image_.is_bigendian    = false;

        left_rect_cam_info_.header = left_rect_image_.header;

        //
        // Continue to publish the rect camera info on the
        // <namespace>/left/camera_info topic for backward compatibility with
        // older versions of the driver
        left_rect_cam_pub_.publish(left_rect_image_, left_rect_cam_info_);

        left_rect_cam_info_pub_.publish(left_rect_cam_info_);

        publishPointCloud(left_rect_frame_id_,
                          points_buff_frame_id_,
                          luma_point_cloud_frame_id_,
                          luma_point_cloud_pub_,
                          luma_point_cloud_,
                          header.width,
                          header.height,
                          header.timeSeconds,
                          header.timeMicroSeconds,
                          luma_cloud_step,
                          points_buff_,
                          &(left_rect_image_.data[0]), luma_color_depth_,
                          pc_max_range_,
                          write_pc_color_packed_,
                          false);

        publishPointCloud(left_rect_frame_id_,
                          points_buff_frame_id_,
                          luma_organized_point_cloud_frame_id_,
                          luma_organized_point_cloud_pub_,
                          luma_organized_point_cloud_,
                          header.width,
                          header.height,
                          header.timeSeconds,
                          header.timeMicroSeconds,
                          luma_cloud_step,
                          points_buff_,
                          &(left_rect_image_.data[0]), luma_color_depth_,
                          pc_max_range_,
                          write_pc_color_packed_,
                          true);

        break;
    case Source_Luma_Rectified_Right:

        right_rect_image_.data.resize(header.imageLength);
        memcpy(&right_rect_image_.data[0], header.imageDataP, header.imageLength);

        right_rect_image_.header.frame_id = frame_id_left_;
        right_rect_image_.header.stamp    = t;
        right_rect_image_.height          = header.height;
        right_rect_image_.width           = header.width;

        switch(header.bitsPerPixel) {
            case 8:
                right_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                right_rect_image_.step     = header.width;
                break;
            case 16:
                right_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                right_rect_image_.step     = header.width * 2;
                break;
        }

        right_rect_image_.is_bigendian    = false;

        right_rect_cam_info_.header = right_rect_image_.header;

        //
        // Continue to publish the rect camera info on the
        // <namespace>/right/camera_info topic for backward compatibility with
        // older versions of the driver
        right_rect_cam_pub_.publish(right_rect_image_, right_rect_cam_info_);

        right_rect_cam_info_pub_.publish(right_rect_cam_info_);

        break;
    }
}

void Camera::depthCallback(const image::Header& header)
{
    if (Source_Disparity != header.source) {

        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }

    uint32_t niDepthSubscribers = ni_depth_cam_pub_.getNumSubscribers();
    uint32_t depthSubscribers = depth_cam_pub_.getNumSubscribers();
    if (0 == niDepthSubscribers &&
        0 == depthSubscribers)
        return;

    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t niDepthSize = header.height * header.width * sizeof(uint16_t);
    const uint32_t imageSize = header.width * header.height;

    depth_image_.header.stamp    = ros::Time(header.timeSeconds,
                                             1000 * header.timeMicroSeconds);
    depth_image_.header.frame_id = frame_id_left_;
    depth_image_.height          = header.height;
    depth_image_.width           = header.width;
    depth_image_.is_bigendian    = (htonl(1) == 1);

    ni_depth_image_ = depth_image_;

    ni_depth_image_.encoding           = sensor_msgs::image_encodings::MONO16;
    ni_depth_image_.step               = header.width * 2;

    depth_image_.encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_image_.step            = header.width * 4;

    depth_image_.data.resize(depthSize);
    ni_depth_image_.data.resize(niDepthSize);

    float *depthImageP = reinterpret_cast<float*>(&depth_image_.data[0]);
    uint16_t *niDepthImageP = reinterpret_cast<uint16_t*>(&ni_depth_image_.data[0]);

    //
    // Disparity is in 32-bit floating point

    if (32 == header.bitsPerPixel) {

        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d).
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.

        const double scale = (-right_rect_cam_info_.P[3]);

        const float *disparityImageP = reinterpret_cast<const float*>(header.imageDataP);

        for (uint32_t i = 0 ; i < imageSize ; ++i)
        {
            if (0.0 >= disparityImageP[i])
            {
                depthImageP[i] = bad_point;
                niDepthImageP[i] = 0;
            }
            else
            {
                depthImageP[i] = scale / disparityImageP[i];
                niDepthImageP[i] = static_cast<uint16_t>(depthImageP[i] * 1000);
            }
        }

    //
    // Disparity is in 1/16th pixel, unsigned integer

    } else if (16 == header.bitsPerPixel) {

        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d). Because our disparity
        // image is 16 bits we must also divide by 16 making z = (fx*Tx*16)/(-d)
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.


        const float scale = (right_rect_cam_info_.P[3] * -16.0f);

        const uint16_t *disparityImageP = reinterpret_cast<const uint16_t*>(header.imageDataP);

        for (uint32_t i = 0 ; i < imageSize ; ++i)
        {
            if (0 == disparityImageP[i])
            {
                depthImageP[i] = bad_point;
                niDepthImageP[i] = 0;
            }
            else
            {
                depthImageP[i] = scale / disparityImageP[i];
                niDepthImageP[i] = static_cast<uint16_t>(depthImageP[i] * 1000);
            }
        }

    } else {
        ROS_ERROR("Camera: unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    if (0 != niDepthSubscribers)
    {
        ni_depth_cam_pub_.publish(ni_depth_image_);
    }

    if (0 != depthSubscribers)
    {
        depth_cam_pub_.publish(depth_image_);
    }

    depth_cam_info_.header = depth_image_.header;
    depth_cam_info_pub_.publish(depth_cam_info_);
}

void Camera::pointCloudCallback(const image::Header& header)
{
    if (Source_Disparity != header.source) {

        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }

    if (0 == luma_point_cloud_pub_.getNumSubscribers() &&
        0 == color_point_cloud_pub_.getNumSubscribers() &&
        0 == luma_organized_point_cloud_pub_.getNumSubscribers() &&
        0 == color_organized_point_cloud_pub_.getNumSubscribers())
        return;

    boost::mutex::scoped_lock lock(border_clip_lock_);

    const bool      handle_missing = true;
    const uint32_t  imageSize      = header.height * header.width;

    //
    // Resize buffers

    points_buff_.resize(imageSize);
    disparity_buff_.resize(imageSize);

    //
    // Allocate buffer for reprojection output

    cv::Mat_<cv::Vec3f> points(header.height, header.width, &(points_buff_[0]));

    //
    // Image is already 32-bit floating point

    if (32 == header.bitsPerPixel) {

        cv::Mat_<float> disparity(header.height, header.width,
                                  const_cast<float*>(reinterpret_cast<const float*>(header.imageDataP)));

        cv::reprojectImageTo3D(disparity, points, q_matrix_, handle_missing);

    //
    // Convert CRL 1/16th pixel disparity to floating point

    } else if (16 == header.bitsPerPixel) {

        cv::Mat_<uint16_t> disparityOrigP(header.height, header.width,
                                          const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(header.imageDataP)));
        cv::Mat_<float>   disparity(header.height, header.width, &(disparity_buff_[0]));
        disparity = disparityOrigP / 16.0f;

        cv::reprojectImageTo3D(disparity, points, q_matrix_, handle_missing);

    } else {
        ROS_ERROR("Camera: unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    //
    // Apply the border clip mask making all the points in the border clip region
    // invalid. Only do this if we have selected a border clip value

    if ( border_clip_value_ > 0)
    {
        points.setTo(cv::Vec3f(10000.0, 10000.0, 10000.0), border_clip_mask_);
    }


    //
    // Store the disparity frame ID

    points_buff_frame_id_ = header.frameId;

    //
    // Publish the point clouds if desired/possible

    publishPointCloud(left_rect_frame_id_,
                      points_buff_frame_id_,
                      luma_point_cloud_frame_id_,
                      luma_point_cloud_pub_,
                      luma_point_cloud_,
                      header.width,
                      header.height,
                      header.timeSeconds,
                      header.timeMicroSeconds,
                      luma_cloud_step,
                      points_buff_,
                      &(left_rect_image_.data[0]), luma_color_depth_,
                      pc_max_range_,
                      write_pc_color_packed_,
                      false);

    publishPointCloud(left_rgb_rect_frame_id_,
                      points_buff_frame_id_,
                      color_point_cloud_frame_id_,
                      color_point_cloud_pub_,
                      color_point_cloud_,
                      header.width,
                      header.height,
                      header.timeSeconds,
                      header.timeMicroSeconds,
                      color_cloud_step,
                      points_buff_,
                      &(left_rgb_rect_image_.data[0]), 3,
                      pc_max_range_,
                      write_pc_color_packed_,
                      false);

    publishPointCloud(left_rect_frame_id_,
                      points_buff_frame_id_,
                      luma_organized_point_cloud_frame_id_,
                      luma_organized_point_cloud_pub_,
                      luma_organized_point_cloud_,
                      header.width,
                      header.height,
                      header.timeSeconds,
                      header.timeMicroSeconds,
                      luma_cloud_step,
                      points_buff_,
                      &(left_rect_image_.data[0]), luma_color_depth_,
                      pc_max_range_,
                      write_pc_color_packed_,
                      true);

    publishPointCloud(left_rgb_rect_frame_id_,
                      points_buff_frame_id_,
                      color_organized_point_cloud_frame_id_,
                      color_organized_point_cloud_pub_,
                      color_organized_point_cloud_,
                      header.width,
                      header.height,
                      header.timeSeconds,
                      header.timeMicroSeconds,
                      color_cloud_step,
                      points_buff_,
                      &(left_rgb_rect_image_.data[0]), 3,
                      pc_max_range_,
                      write_pc_color_packed_,
                      true);
}

void Camera::rawCamDataCallback(const image::Header& header)
{
    if (0 == raw_cam_data_pub_.getNumSubscribers()) {
        got_raw_cam_left_ = false;
        return;
    }

    const uint32_t imageSize = header.width * header.height;

    //
    // The left-rectified image is currently published before
    // the matching disparity image.

    if (false == got_raw_cam_left_) {

        if (Source_Luma_Rectified_Left == header.source) {

            raw_cam_data_.gray_scale_image.resize(imageSize);
            memcpy(&(raw_cam_data_.gray_scale_image[0]),
                   header.imageDataP,
                   imageSize * sizeof(uint8_t));

            raw_cam_data_.frames_per_second = header.framesPerSecond;
            raw_cam_data_.gain              = header.gain;
            raw_cam_data_.exposure_time     = header.exposure;
            raw_cam_data_.frame_count       = header.frameId;
            raw_cam_data_.time_stamp        = ros::Time(header.timeSeconds,
                                                        1000 * header.timeMicroSeconds);
            raw_cam_data_.width             = header.width;
            raw_cam_data_.height            = header.height;

            got_raw_cam_left_ = true;
        }

    } else if (Source_Disparity == header.source) {

        const uint32_t imageSize = header.width * header.height;

        if (header.frameId == raw_cam_data_.frame_count) {

            raw_cam_data_.disparity_image.resize(imageSize);
            memcpy(&(raw_cam_data_.disparity_image[0]),
                   header.imageDataP, imageSize * sizeof(uint16_t));

            raw_cam_data_pub_.publish(raw_cam_data_);
        }

        got_raw_cam_left_ = false;
    }
}

void Camera::colorImageCallback(const image::Header& header)
{
    if (0 == left_rgb_cam_pub_.getNumSubscribers() &&
        0 == left_rgb_rect_cam_pub_.getNumSubscribers() &&
        0 == color_point_cloud_pub_.getNumSubscribers() &&
        0 == color_organized_point_cloud_pub_.getNumSubscribers()) {
        got_left_luma_ = false;
        return;
    }

    //
    // The left-luma image is currently published before
    // the matching chroma image.

    if (false == got_left_luma_) {

        if (Source_Luma_Left == header.source) {

            const uint32_t imageSize = header.width * header.height;

            left_luma_image_.data.resize(imageSize);
            memcpy(&left_luma_image_.data[0], header.imageDataP, imageSize);

            left_luma_image_.height = header.height;
            left_luma_image_.width  = header.width;

            left_luma_frame_id_ = header.frameId;
            got_left_luma_      = true;
        }

    } else if (Source_Chroma_Left == header.source) {

        if (header.frameId == left_luma_frame_id_) {

            const uint32_t height    = left_luma_image_.height;
            const uint32_t width     = left_luma_image_.width;
            const uint32_t imageSize = 3 * height * width;

            left_rgb_image_.data.resize(imageSize);

            left_rgb_image_.header.frame_id = frame_id_left_;
            left_rgb_image_.header.stamp    = ros::Time(header.timeSeconds,
                                                        1000 * header.timeMicroSeconds);
            left_rgb_image_.height          = height;
            left_rgb_image_.width           = width;

            left_rgb_image_.encoding        = "bgr8";
            left_rgb_image_.is_bigendian    = false;
            left_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB
            // TODO: speed this up

            const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(&(left_luma_image_.data[0]));
            const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(header.imageDataP);
            uint8_t       *bgrP      = reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0]));
            const uint32_t rgbStride = width * 3;

            for(uint32_t y=0; y<height; y++) {
                for(uint32_t x=0; x<width; x++) {

                    const uint32_t lumaOffset   = (y * width) + x;
                    const uint32_t chromaOffset = 2 * (((y/2) * (width/2)) + (x/2));

                    const float px_y  = static_cast<float>(lumaP[lumaOffset]);
                    const float px_cb = static_cast<float>(chromaP[chromaOffset+0]) - 128.0f;
                    const float px_cr = static_cast<float>(chromaP[chromaOffset+1]) - 128.0f;

                    float px_r  = px_y +                    1.402f   * px_cr;
                    float px_g  = px_y - 0.34414f * px_cb - 0.71414f * px_cr;
                    float px_b  = px_y + 1.772f   * px_cb;

                    if (px_r < 0.0f)        px_r = 0.0f;
                    else if (px_r > 255.0f) px_r = 255.0f;
                    if (px_g < 0.0f)        px_g = 0.0f;
                    else if (px_g > 255.0f) px_g = 255.0f;
                    if (px_b < 0.0f)        px_b = 0.0f;
                    else if (px_b > 255.0f) px_b = 255.0f;

                    const uint32_t rgbOffset = (y * rgbStride) + (3 * x);

                    bgrP[rgbOffset + 0] = static_cast<uint8_t>(px_b);
                    bgrP[rgbOffset + 1] = static_cast<uint8_t>(px_g);
                    bgrP[rgbOffset + 2] = static_cast<uint8_t>(px_r);
                }
            }

            if (0 != left_rgb_cam_pub_.getNumSubscribers()) {
                left_rgb_cam_pub_.publish(left_rgb_image_);

                left_rgb_cam_info_.header = left_rgb_image_.header;
                left_rgb_cam_info_pub_.publish(left_rgb_cam_info_);
            }

            if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0 ||
                color_point_cloud_pub_.getNumSubscribers() > 0 ||
                color_organized_point_cloud_pub_.getNumSubscribers() > 0) {
                boost::mutex::scoped_lock lock(cal_lock_);

                if (width  != image_config_.width() ||
                    height != image_config_.height())
                    //ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
                    //width, height, image_config_.width(), image_config_.height());
                    ;
                else if (calibration_map_left_1_.empty() || calibration_map_left_2_.empty())
                    ROS_ERROR("Camera: undistort maps not initialized");
                else {
                    left_rgb_rect_image_.data.resize(imageSize);

                    const cv::Mat rgb_image(height, width, CV_8UC3, &(left_rgb_image_.data[0]));
                    cv::Mat rect_rgb_image(height, width, CV_8UC3, &(left_rgb_rect_image_.data[0]));

                    cv::remap(rgb_image, rect_rgb_image,
                            calibration_map_left_1_,
                            calibration_map_left_2_,
                            cv::INTER_CUBIC);

                    left_rgb_rect_image_.header.frame_id = frame_id_left_;
                    left_rgb_rect_image_.header.stamp    = ros::Time(header.timeSeconds,
                                                                     1000 * header.timeMicroSeconds);
                    left_rgb_rect_image_.height          = height;
                    left_rgb_rect_image_.width           = width;

                    left_rgb_rect_image_.encoding        = "bgr8";
                    left_rgb_rect_image_.is_bigendian    = false;
                    left_rgb_rect_image_.step            = 3 * width;

                    left_rgb_rect_cam_info_.header = left_rgb_rect_image_.header;
                    left_rgb_rect_frame_id_              = header.frameId;

                    if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0) {
                        left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rgb_rect_cam_info_);

                        left_rgb_rect_cam_info_.header = left_rgb_rect_image_.header;
                        left_rgb_rect_cam_info_pub_.publish(left_rgb_rect_cam_info_);
                    }

                    //
                    // Publish the color point cloud if desired/possible

                    publishPointCloud(left_rgb_rect_frame_id_,
                                      points_buff_frame_id_,
                                      color_point_cloud_frame_id_,
                                      color_point_cloud_pub_,
                                      color_point_cloud_,
                                      left_luma_image_.width,
                                      left_luma_image_.height,
                                      header.timeSeconds,
                                      header.timeMicroSeconds,
                                      color_cloud_step,
                                      points_buff_,
                                      &(left_rgb_rect_image_.data[0]), 3,
                                      pc_max_range_,
                                      write_pc_color_packed_,
                                      false);

                    publishPointCloud(left_rgb_rect_frame_id_,
                                      points_buff_frame_id_,
                                      color_organized_point_cloud_frame_id_,
                                      color_organized_point_cloud_pub_,
                                      color_organized_point_cloud_,
                                      left_luma_image_.width,
                                      left_luma_image_.height,
                                      header.timeSeconds,
                                      header.timeMicroSeconds,
                                      color_cloud_step,
                                      points_buff_,
                                      &(left_rgb_rect_image_.data[0]), 3,
                                      pc_max_range_,
                                      write_pc_color_packed_,
                                      true);
                }
            }
        }

        got_left_luma_ = false;
    }
}

void Camera::queryConfig()
{
    boost::mutex::scoped_lock lock(cal_lock_);

    //
    // Get the camera config

    Status status = driver_->getImageConfig(image_config_);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query sensor configuration: %s",
                  Channel::statusString(status));
        return;
    }

    //
    // For convenience

    const image::Config& c = image_config_;

    disparities_ = c.disparities();

    //
    // Frame IDs must match for the rectified images

    left_rect_cam_info_.header.frame_id  = frame_id_left_;
    left_rect_cam_info_.header.stamp     = ros::Time::now();
    right_rect_cam_info_.header          = left_rect_cam_info_.header;

    left_rect_cam_info_.width  = c.width();
    left_rect_cam_info_.height = c.height();

    right_rect_cam_info_.width = c.width();
    right_rect_cam_info_.height = c.height();

    //
    // Calibration from sensor is for native resolution

    image::Calibration cal = image_calibration_;

    const float x_scale = 1.0f / ((static_cast<float>(device_info_.imagerWidth) /
                                   static_cast<float>(c.width())));
    const float y_scale = 1.0f / ((static_cast<float>(device_info_.imagerHeight) /
                                   static_cast<float>(c.height())));

    //
    // Populate our rectified camera info topics

    updateCameraInfo(left_rect_cam_info_, cal.left.M, cal.left.R, cal.left.P, cal.left.D, x_scale, y_scale);
    updateCameraInfo(right_rect_cam_info_, cal.right.M, cal.right.R, cal.right.P, cal.right.D, x_scale, y_scale);

    //
    // Copy our rectified camera info topics to populate the other camera info.

    left_disp_cam_info_ = left_rect_cam_info_;
    left_cost_cam_info_ = left_rect_cam_info_;
    left_rgb_rect_cam_info_ = left_rect_cam_info_;
    left_mono_cam_info_ = left_rect_cam_info_;
    left_rgb_cam_info_ = left_rect_cam_info_;
    depth_cam_info_ = left_rect_cam_info_;

    right_mono_cam_info_ = right_rect_cam_info_;
    right_disp_cam_info_ = right_rect_cam_info_;

    //
    // Update the frame ID's for the unrectified right image topics

    right_mono_cam_info_.header.frame_id = frame_id_right_;
    right_disp_cam_info_.header.frame_id = frame_id_right_;

    //
    // Compute the Q matrix here, as image_geometery::StereoCameraModel does
    // not allow for non-square pixels.
    //
    //  FyTx    0     0    -FyCxTx
    //   0     FxTx   0    -FxCyTx
    //   0      0     0     FxFyTx
    //   0      0    -Fy    Fy(Cx - Cx')

    q_matrix_(0,0) =  c.fy() * c.tx();
    q_matrix_(1,1) =  c.fx() * c.tx();
    q_matrix_(0,3) = -c.fy() * c.cx() * c.tx();
    q_matrix_(1,3) = -c.fx() * c.cy() * c.tx();
    q_matrix_(2,3) =  c.fx() * c.fy() * c.tx();
    q_matrix_(3,2) = -c.fy();
    q_matrix_(3,3) =  c.fy() * (right_rect_cam_info_.P[2] - left_rect_cam_info_.P[2]);

    //
    // Create rectification maps for local rectification of color images

    calibration_map_left_1_ = cv::Mat(c.height(), c.width(), CV_32F);
    calibration_map_left_2_ = cv::Mat(c.height(), c.width(), CV_32F);

    cal.left.M[0][0]  *= x_scale;  cal.left.M[1][1]  *= y_scale;
    cal.left.M[0][2]  *= x_scale;  cal.left.M[1][2]  *= y_scale;
    cal.right.M[0][0] *= x_scale;  cal.right.M[1][1] *= y_scale;
    cal.right.M[0][2] *= x_scale;  cal.right.M[1][2] *= y_scale;
    cal.left.P[0][0]  *= x_scale;  cal.left.P[1][1]  *= y_scale;
    cal.left.P[0][2]  *= x_scale;  cal.left.P[1][2]  *= y_scale;
    cal.left.P[0][3]  *= x_scale;  cal.left.P[1][3]  *= y_scale;

    cv::Mat M1 = cv::Mat(3, 3, CV_32F, &cal.left.M);
    cv::Mat D1 = cv::Mat(1, 8, CV_32F, &cal.left.D);
    cv::Mat R1 = cv::Mat(3, 3, CV_32F, &cal.left.R);
    cv::Mat P1 = cv::Mat(3, 4, CV_32F, &cal.left.P);

    cv::initUndistortRectifyMap(M1, D1, R1, P1,
                                cv::Size(c.width(), c.height()),
                                CV_32F,
                                calibration_map_left_1_,
                                calibration_map_left_2_);


    //
    // Publish the "raw" config message

    multisense_ros::RawCamConfig cfg;

    cfg.width             = c.width();
    cfg.height            = c.height();
    cfg.frames_per_second = c.fps();
    cfg.gain              = c.gain();
    cfg.exposure_time     = c.exposure();

    cfg.fx    = c.fx();
    cfg.fy    = c.fy();
    cfg.cx    = c.cx();
    cfg.cy    = c.cy();
    cfg.tx    = c.tx();
    cfg.ty    = c.ty();
    cfg.tz    = c.tz();
    cfg.roll  = c.roll();
    cfg.pitch = c.pitch();
    cfg.yaw   = c.yaw();

    raw_cam_config_pub_.publish(cfg);

    //
    // Update the border clipping mask since the resolution changed

    generateBorderClip(border_clip_type_, border_clip_value_, c.height(), c.width());

    //
    // Republish our camera info topics since the resolution changed

    publishAllCameraInfo();
}

void Camera::updateCameraInfo(sensor_msgs::CameraInfo& cameraInfo,
                              const float M[3][3],
                              const float R[3][3],
                              const float P[3][4],
                              const float D[8],
                              double xScale,
                              double yScale
)
{
    cameraInfo.P[0]   = P[0][0] * xScale;      cameraInfo.P[1]   = P[0][1];
    cameraInfo.P[2]   = P[0][2] * xScale;      cameraInfo.P[3]   = P[0][3] * xScale;
    cameraInfo.P[4]   = P[1][0];               cameraInfo.P[5]   = P[1][1] * yScale;
    cameraInfo.P[6]   = P[1][2] * yScale;      cameraInfo.P[7]   = P[1][3];
    cameraInfo.P[8]   = P[2][0];               cameraInfo.P[9]   = P[2][1];
    cameraInfo.P[10]  = P[2][2];               cameraInfo.P[11]  = P[2][3];

    cameraInfo.K[0]   = M[0][0] * xScale;      cameraInfo.K[1]   = M[0][1];
    cameraInfo.K[2]   = M[0][2] * xScale;      cameraInfo.K[3]   = M[1][0];
    cameraInfo.K[4]   = M[1][1] * yScale;      cameraInfo.K[5]   = M[1][2] * yScale;
    cameraInfo.K[6]   = M[2][0];               cameraInfo.K[7]   = M[2][1];
    cameraInfo.K[8]   = M[2][2];

    cameraInfo.R[0] = R[0][0];                 cameraInfo.R[1] = R[0][1];
    cameraInfo.R[2] = R[0][2];                 cameraInfo.R[3] = R[1][0];
    cameraInfo.R[4] = R[1][1];                 cameraInfo.R[5] = R[1][2];
    cameraInfo.R[6] = R[2][0];                 cameraInfo.R[7] = R[2][1];
    cameraInfo.R[8] = R[2][2];

    //
    // Distortion coefficients follow OpenCV's convention.
    // k1, k2, p1, p2, k3, k4, k5, k6

    cameraInfo.D.resize(8);
    for (uint8_t i=0 ; i < 8 ; ++i) {
        cameraInfo.D[i] = D[i];
    }

    //
    // MultiSense cameras support both the full 8 parameter rational_polynomial
    // model and the simplified 5 parameter plum_bob model. If the last 3
    // parameters of the distortion model are 0 then the camera is using
    // the simplified plumb_bob model

    if (D[7] == 0.0 && D[6] == 0.0 && D[5] == 0.0) {
        cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    } else {
        cameraInfo.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
    }

}


void Camera::publishAllCameraInfo()
{

    //
    // Republish camera info messages outside of image callbacks.
    // The camera info publishers are latching so the messages
    // will persist until a new message is published in one of the image
    // callbacks. This makes it easier when a user is trying access a camera_info
    // for a topic which they are not subscribed to

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        left_mono_cam_info_pub_.publish(left_mono_cam_info_);
        left_rgb_cam_info_pub_.publish(left_rgb_cam_info_);
        left_rgb_rect_cam_info_pub_.publish(left_rgb_rect_cam_info_);

    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_M == device_info_.hardwareRevision) {

        left_mono_cam_info_pub_.publish(left_mono_cam_info_);
        left_rect_cam_info_pub_.publish(left_rect_cam_info_);
        left_rgb_cam_info_pub_.publish(left_rgb_cam_info_);
        left_rgb_rect_cam_info_pub_.publish(left_rgb_rect_cam_info_);

    } else {  // all other MultiSense-S* variations

        if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 != device_info_.hardwareRevision) {

            left_rgb_cam_info_pub_.publish(left_rgb_cam_info_);
            left_rgb_rect_cam_info_pub_.publish(left_rgb_rect_cam_info_);
        }

        if (version_info_.sensorFirmwareVersion >= 0x0300) {

            right_disp_cam_info_pub_.publish(right_disp_cam_info_);
            left_cost_cam_info_pub_.publish(left_cost_cam_info_);
        }

        left_mono_cam_info_pub_.publish(left_mono_cam_info_);
        left_rect_cam_info_pub_.publish(left_rect_cam_info_);
        right_mono_cam_info_pub_.publish(right_mono_cam_info_);
        right_rect_cam_info_pub_.publish(right_rect_cam_info_);
        left_disp_cam_info_pub_.publish(left_disp_cam_info_);
        depth_cam_info_pub_.publish(depth_cam_info_);

    }
}

void Camera::borderClipChanged(int borderClipType, double borderClipValue)
{
    //
    // This assumes the image resolution did not change and will just use
    // the current mask size as width and height arguments

    generateBorderClip(borderClipType, borderClipValue, border_clip_mask_.rows, border_clip_mask_.cols);

}

void Camera::generateBorderClip(int borderClipType, double borderClipValue, uint32_t height, uint32_t width)
{

    boost::mutex::scoped_lock lock(border_clip_lock_);

    border_clip_type_ = borderClipType;
    border_clip_value_ = borderClipValue;

    //
    // Reset the border clip mask

    border_clip_mask_ = cv::Mat_<uint8_t>(height, width, static_cast<uint8_t>(255));

    //
    // Manually generate our disparity border clipping mask. Points with
    // a value of 255 are excluded from the pointcloud. Points with a value of 0
    // are included

    double halfWidth = static_cast<double>(width)/2.0;
    double halfHeight = static_cast<double>(height)/2.0;

    //
    // Precompute the maximum radius from the center of the image for a point
    // to be considered in the circle

    double radius = sqrt( pow( halfWidth, 2) + pow( halfHeight, 2) );
    radius -= borderClipValue;

    for (uint32_t u = 0 ; u < width ; ++u)
    {
        for (uint32_t v = 0 ; v < height ; ++v)
        {
            switch (borderClipType)
            {
                case RECTANGULAR:
                {
                    if ( u >= borderClipValue && u <= width - borderClipValue &&
                         v >= borderClipValue && v <= height - borderClipValue)
                    {
                        border_clip_mask_(v, u) = 0;
                    }

                    break;
                }
                case CIRCULAR:
                {
                    double vector = sqrt( pow( halfWidth - u, 2) +
                                          pow( halfHeight - v, 2) );

                    if ( vector < radius)
                    {
                        border_clip_mask_(v, u) = 0;
                    }

                    break;
                }
                default:
                {
                    ROS_WARN("Unknown border clip type.");
                    return;
                }
            }
        }
    }
}

void Camera::stop()
{
    boost::mutex::scoped_lock lock(stream_lock_);

    stream_map_.clear();

    Status status = driver_->stopStreams(allImageSources);
    if (Status_Ok != status)
        ROS_ERROR("Camera: failed to stop all streams: %s",
                  Channel::statusString(status));
}

void Camera::connectStream(DataSource enableMask)
{
    boost::mutex::scoped_lock lock(stream_lock_);

    DataSource notStarted = 0;

    for(uint32_t i=0; i<32; i++)
        if ((1<<i) & enableMask && 0 == stream_map_[(1<<i)]++)
            notStarted |= (1<<i);

    if (0 != notStarted) {

        Status status = driver_->startStreams(notStarted);
        if (Status_Ok != status)
            ROS_ERROR("Camera: failed to start streams 0x%x: %s",
                      notStarted, Channel::statusString(status));
    }
}

void Camera::disconnectStream(DataSource disableMask)
{
    boost::mutex::scoped_lock lock(stream_lock_);

    DataSource notStopped = 0;

    for(uint32_t i=0; i<32; i++)
        if ((1<<i) & disableMask && 0 == --stream_map_[(1<<i)])
            notStopped |= (1<<i);

    if (0 != notStopped) {
        Status status = driver_->stopStreams(notStopped);
        if (Status_Ok != status)
            ROS_ERROR("Camera: failed to stop streams 0x%x: %s\n",
                      notStopped, Channel::statusString(status));
    }
}



} // namespace
