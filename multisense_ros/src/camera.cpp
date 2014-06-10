/**
 * @file camera.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#include <multisense_ros/camera.h>
#include <multisense_ros/RawCamConfig.h>
#include <multisense_ros/RawCamCal.h>
#include <multisense_ros/DeviceInfo.h>

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

const uint32_t luma_cloud_step  = 13;
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

void publishPointCloud(ros::Publisher&               pub,
                       sensor_msgs::PointCloud2&     cloud,
                       const image::Header&          header,
                       const uint32_t                cloudStep,
                       const std::vector<cv::Vec3f>& points,
                       const uint8_t*                imageP,
                       const uint32_t                colorChannels,
                       const uint32_t                borderClip,
                       const float                   maxRange)
{
    if (0 == pub.getNumSubscribers())
        return;

    const uint32_t imageSize = header.height * header.width;

    if (points.size() != imageSize)
        return;

    cloud.data.reserve(imageSize * cloudStep);
    
    uint8_t       *cloudP      = reinterpret_cast<uint8_t*>(&cloud.data[0]);
    const uint32_t pointSize   = 3 * sizeof(float); // x, y, z
    uint32_t       validPoints = 0;

    for(uint32_t i=borderClip; i<(header.height - borderClip); ++i)
        for(uint32_t j=borderClip; j<(header.width - borderClip); ++j) {

            const uint32_t index = i*header.width + j;

            if (false == isValidPoint(points[index], maxRange))
                continue;

            memcpy(cloudP, &(points[index]), pointSize);
                
            const uint8_t *sourceColorP = &(imageP[colorChannels * index]);
            uint8_t       *cloudColorP  = (cloudP + pointSize);

            //
            // For color: image stored as BGR, index image RGB.
                
            for(uint32_t k=0; k<colorChannels; k++)
                cloudColorP[k] = sourceColorP[colorChannels - k - 1];
            
            cloudP += cloudStep;
            validPoints ++;
        }

    cloud.row_step     = validPoints;
    cloud.width        = validPoints;
    cloud.header.stamp = ros::Time(header.timeSeconds,
                                   1000 * header.timeMicroSeconds);
    cloud.data.resize(validPoints * cloudStep);
    pub.publish(cloud);
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
    disparity_left_transport_(left_nh_),
    disparity_right_transport_(right_nh_),
    disparity_cost_transport_(left_nh_),
    left_rect_cam_info_(),
    right_rect_cam_info_(),
    left_rgb_rect_cam_info_(),
    left_mono_cam_pub_(),
    right_mono_cam_pub_(),
    left_rect_cam_pub_(),
    right_rect_cam_pub_(),
    depth_cam_pub_(),
    left_rgb_cam_pub_(),
    left_rgb_rect_cam_pub_(),
    luma_point_cloud_pub_(),
    color_point_cloud_pub_(),
    left_disparity_pub_(),
    right_disparity_pub_(),
    left_disparity_cost_pub_(),
    raw_cam_data_pub_(),
    raw_cam_config_pub_(),
    raw_cam_cal_pub_(),
    device_info_pub_(),
    left_mono_image_(),
    right_mono_image_(),
    left_rect_image_(),
    right_rect_image_(),
    depth_image_(),
    luma_point_cloud_(),
    color_point_cloud_(),
    left_luma_image_(),
    left_rgb_image_(),
    left_rgb_rect_image_(),
    left_disparity_image_(),
    left_disparity_cost_image_(),
    right_disparity_image_(),
    got_raw_cam_left_(false),
    got_left_luma_(false),
    left_luma_frame_id_(0),
    left_rect_frame_id_(0),
    left_rgb_rect_frame_id_(0),
    raw_cam_data_(),
    version_info_(),
    device_info_(),
    image_config_(),
    image_calibration_(),
    cal_lock_(),
    calibration_map_left_1_(NULL),
    calibration_map_left_2_(NULL),
    frame_id_left_(),
    frame_id_right_(),
    disparity_buff_(),
    points_buff_(),
    q_matrix_(4, 4, 0.0),
    pc_border_clip_(10),
    pc_max_range_(15.0f),
    pc_color_frame_sync_(true),
    stream_lock_(),
    stream_map_()
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

    //
    // Create topic publishers (TODO: color topics should not be advertised if the device can't support it)

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        left_mono_cam_pub_  = left_mono_transport_.advertise("image_mono", 5, 
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        /*
        left_rect_cam_pub_  = left_rect_transport_.advertiseCamera("image_rect", 5, 
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        */
        left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));
        
        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));

    } else {  // all MultiSense-S* variations


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
        depth_cam_pub_      = depth_transport_.advertiseCamera("depth", 5, 
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Disparity));
        left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
        luma_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("image_points2", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));
        color_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>("image_points2_color", 5,
                              boost::bind(&Camera::connectStream, this, 
                              Source_Disparity | Source_Luma_Left | Source_Chroma_Left),
                              boost::bind(&Camera::disconnectStream, this, 
                              Source_Disparity | Source_Luma_Left | Source_Chroma_Left));

        raw_cam_data_pub_   = calibration_nh.advertise<multisense_ros::RawCamData>("raw_cam_data", 5,
                              boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        left_disparity_pub_ = disparity_left_transport_.advertise("disparity", 5,
                              boost::bind(&Camera::connectStream, this, Source_Disparity),
                             boost::bind(&Camera::disconnectStream, this, Source_Disparity));

        if (version_info_.sensorFirmwareVersion >= 0x0300) {

            right_disparity_pub_ = disparity_right_transport_.advertise("disparity", 5,
                                   boost::bind(&Camera::connectStream, this, Source_Disparity_Right),
                                   boost::bind(&Camera::disconnectStream, this, Source_Disparity_Right));
            left_disparity_cost_pub_ = disparity_cost_transport_.advertise("cost", 5,
                                   boost::bind(&Camera::connectStream, this, Source_Disparity_Cost),
                                   boost::bind(&Camera::disconnectStream, this, Source_Disparity_Cost));
        }
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
    luma_point_cloud_.fields[3].datatype = sensor_msgs::PointField::UINT8;

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
    color_point_cloud_.fields[3].datatype = sensor_msgs::PointField::UINT32;

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
    // Get the border clip, if any

    const char *pcBorderClipEnvStringP = getenv("MULTISENSE_ROS_PC_BORDER_CLIP");
    if (NULL != pcBorderClipEnvStringP) {
        int32_t tmp = atoi(pcBorderClipEnvStringP);
        if (tmp > 0) {
            pc_border_clip_ = atoi(pcBorderClipEnvStringP);
            ROS_INFO("pc_border_clip: %d", pc_border_clip_);
        }
    }

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

    if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0) {
        boost::mutex::scoped_lock lock(cal_lock_);
        
        if (width  != image_config_.width() ||
            height != image_config_.height())
            //ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
            //width, height, image_config_.width(), image_config_.height());
            ;
        else if (NULL == calibration_map_left_1_ || NULL == calibration_map_left_2_)
            ROS_ERROR("Camera: undistort maps not initialized");
        else {

            const CvScalar outlierColor = {{0.0}};
            
            left_rgb_rect_image_.data.resize(rgbLength);

            IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
            sourceImageP->imageData = reinterpret_cast<char*>(&(left_rgb_image_.data[0]));
            IplImage *destImageP    = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
            destImageP->imageData   = reinterpret_cast<char*>(&(left_rgb_rect_image_.data[0]));
            
            cvRemap(sourceImageP, destImageP, 
                    calibration_map_left_1_, 
                    calibration_map_left_2_,
                    CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, 
                    outlierColor);

            cvReleaseImageHeader(&sourceImageP);
            cvReleaseImageHeader(&destImageP);

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
           left_disparity_cost_pub_.getNumSubscribers() > 0)))
        return;

    const uint32_t imageSize = (header.width * header.height * header.bitsPerPixel) / 8;

    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Disparity:
    case Source_Disparity_Right:
    {
        sensor_msgs::Image         *imageP=NULL;
        image_transport::Publisher *pubP  =NULL;

        if (Source_Disparity == header.source) {
            pubP                    = &left_disparity_pub_;
            imageP                  = &left_disparity_image_;
            imageP->header.frame_id = frame_id_left_;
        } else {
            pubP                    = &right_disparity_pub_;
            imageP                  = &right_disparity_image_;
            imageP->header.frame_id = frame_id_right_;
        }

        imageP->data.resize(imageSize);
        memcpy(&imageP->data[0], header.imageDataP, imageSize);

        imageP->header.stamp    = t;
        imageP->height          = header.height;
        imageP->width           = header.width;
        imageP->is_bigendian    = false;
        imageP->step            = header.width;

        switch(header.bitsPerPixel) {
        case 8:  imageP->encoding = "mono8";  break;
        case 16: imageP->encoding = "mono16"; break;
        }

        pubP->publish(*imageP);

        break;
    }
    case Source_Disparity_Cost:

        left_disparity_cost_image_.data.resize(imageSize);
        memcpy(&left_disparity_cost_image_.data[0], header.imageDataP, imageSize);

        left_disparity_cost_image_.header.frame_id = frame_id_left_;
        left_disparity_cost_image_.header.stamp    = t;
        left_disparity_cost_image_.height          = header.height;
        left_disparity_cost_image_.width           = header.width;
        
        left_disparity_cost_image_.encoding        = "mono8";
        left_disparity_cost_image_.is_bigendian    = false;
        left_disparity_cost_image_.step            = header.width;

        left_disparity_cost_pub_.publish(left_disparity_cost_image_);

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

    const uint32_t imageSize = header.width * header.height;

    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Left:

        left_mono_image_.data.resize(imageSize);
        memcpy(&left_mono_image_.data[0], header.imageDataP, imageSize);

        left_mono_image_.header.frame_id = frame_id_left_;
        left_mono_image_.header.stamp    = t;
        left_mono_image_.height          = header.height;
        left_mono_image_.width           = header.width;
        
        left_mono_image_.encoding        = "mono8";
        left_mono_image_.is_bigendian    = false;
        left_mono_image_.step            = header.width;

        left_mono_cam_pub_.publish(left_mono_image_);

        break;
    case Source_Luma_Right:

        right_mono_image_.data.resize(imageSize);
        memcpy(&right_mono_image_.data[0], header.imageDataP, imageSize);

        right_mono_image_.header.frame_id = frame_id_right_;
        right_mono_image_.header.stamp    = t;
        right_mono_image_.height          = header.height;
        right_mono_image_.width           = header.width;
        
        right_mono_image_.encoding        = "mono8";
        right_mono_image_.is_bigendian    = false;
        right_mono_image_.step            = header.width;
        
        right_mono_cam_pub_.publish(right_mono_image_);

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

    const uint32_t imageSize = header.width * header.height;

    ros::Time t = ros::Time(header.timeSeconds,
                            1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Rectified_Left:

        left_rect_image_.data.resize(imageSize);
        memcpy(&left_rect_image_.data[0], header.imageDataP, imageSize);

        left_rect_image_.header.frame_id = frame_id_left_;
        left_rect_image_.header.stamp    = t;
        left_rect_image_.height          = header.height;
        left_rect_image_.width           = header.width;

        left_rect_frame_id_              = header.frameId;
        
        left_rect_image_.encoding        = "mono8";
        left_rect_image_.is_bigendian    = false;
        left_rect_image_.step            = header.width;
        
        left_rect_cam_info_.header = left_rect_image_.header;

        left_rect_cam_pub_.publish(left_rect_image_, left_rect_cam_info_);

        break;
    case Source_Luma_Rectified_Right:

        right_rect_image_.data.resize(imageSize);
        memcpy(&right_rect_image_.data[0], header.imageDataP, imageSize);

        right_rect_image_.header.frame_id = frame_id_left_;
        right_rect_image_.header.stamp    = t;
        right_rect_image_.height          = header.height;
        right_rect_image_.width           = header.width;
        
        right_rect_image_.encoding        = "mono8";
        right_rect_image_.is_bigendian    = false;
        right_rect_image_.step            = header.width;
        
        right_rect_cam_info_.header = right_rect_image_.header;
        
        right_rect_cam_pub_.publish(right_rect_image_, right_rect_cam_info_);

        break;
    }
}

void Camera::depthCallback(const image::Header& header)
{
    if (Source_Disparity != header.source) {
     
        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }

    if (0 == depth_cam_pub_.getNumSubscribers())
        return;

    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t imageSize = header.width * header.height;

    depth_image_.header.stamp    = ros::Time(header.timeSeconds,
                                             1000 * header.timeMicroSeconds);
    depth_image_.header.frame_id = frame_id_left_;
    depth_image_.height          = header.height;
    depth_image_.width           = header.width;
    depth_image_.encoding        = "32FC1";
    depth_image_.is_bigendian    = (htonl(1) == 1);
    depth_image_.step            = header.width * 4;
    
    depth_image_.data.resize(depthSize);
    
    float *depthImageP = reinterpret_cast<float*>(&depth_image_.data[0]);

    cv::Mat_<float> depth(header.height, header.width, 
                          reinterpret_cast<float*>(&depth_image_.data[0]));

    //
    // Disparity is in 32-bit floating point

    if (32 == header.bitsPerPixel) {

        cv::Mat_<float> disparity(header.height, header.width, 
                                  const_cast<float*>(reinterpret_cast<const float*>(header.imageDataP)));
        
        //
        // Depth = focal_length*baseline/disparity

        const double scale = (right_rect_cam_info_.P[3] * right_rect_cam_info_.P[0]);
        cv::divide(scale, disparity, depth);

        //
        // Mark all 0 disparity points as NaNs

        const float *disparityImageP = reinterpret_cast<const float*>(header.imageDataP);

        for(uint32_t i=0; i<imageSize; ++i)
            if (0.0 == disparityImageP[i])
                depthImageP[i] = bad_point;

    //
    // Disparity is in 1/16th pixel, unsigned integer

    } else if (16 == header.bitsPerPixel) {

        cv::Mat_<uint16_t> disparity(header.height, header.width, 
                                     const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(header.imageDataP)));
        
        //
        // Depth = focal_length*baseline/disparity

        const float scale = ((right_rect_cam_info_.P[3] * right_rect_cam_info_.P[0]) / -16.0f);
        cv::divide(scale, disparity, depth);

        //
        // Mark all 0 disparity points as NaNs

        const uint16_t *disparityImageP = reinterpret_cast<const uint16_t*>(header.imageDataP);

        for(uint32_t i=0; i<imageSize; ++i)
            if (0 == disparityImageP[i])
                depthImageP[i] = bad_point;

    } else {
        ROS_ERROR("Camera: unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    depth_cam_pub_.publish(depth_image_, left_rect_cam_info_);
}

void Camera::pointCloudCallback(const image::Header& header)
{    
    if (Source_Disparity != header.source) {
     
        ROS_ERROR("Camera: unexpected image source: 0x%x", header.source);
        return;
    }

    if (0 == luma_point_cloud_pub_.getNumSubscribers() &&
        0 == color_point_cloud_pub_.getNumSubscribers())
        return;
       
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
    // Publish the cloud if we have a matching luma image

    if (left_rect_frame_id_ == header.frameId)
        publishPointCloud(luma_point_cloud_pub_,
                          luma_point_cloud_,
                          header,
                          luma_cloud_step,
                          points_buff_,
                          &(left_rect_image_.data[0]), 1,
                          pc_border_clip_,
                          pc_max_range_);

    if (false == pc_color_frame_sync_ || left_rgb_rect_frame_id_ == header.frameId)
        publishPointCloud(color_point_cloud_pub_,
                          color_point_cloud_,
                          header,
                          color_cloud_step,
                          points_buff_,
                          &(left_rgb_rect_image_.data[0]), 3,
                          pc_border_clip_,
                          pc_max_range_);
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
        0 == color_point_cloud_pub_.getNumSubscribers()) {
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
             
            left_rgb_image_.encoding        = "rgb8";
            left_rgb_image_.is_bigendian    = false;
            left_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB
            // TODO: speed this up

            const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(&(left_luma_image_.data[0]));
            const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(header.imageDataP);
            uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0]));
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

                    rgbP[rgbOffset + 0] = static_cast<uint8_t>(px_r);
                    rgbP[rgbOffset + 1] = static_cast<uint8_t>(px_g);
                    rgbP[rgbOffset + 2] = static_cast<uint8_t>(px_b);
                }
            }

            if (0 != left_rgb_cam_pub_.getNumSubscribers())
                left_rgb_cam_pub_.publish(left_rgb_image_);

            if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0 ||
                color_point_cloud_pub_.getNumSubscribers() > 0) {
                boost::mutex::scoped_lock lock(cal_lock_);

                if (width  != image_config_.width() ||
                    height != image_config_.height())
                    //ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
                    //width, height, image_config_.width(), image_config_.height());
                    ;
                else if (NULL == calibration_map_left_1_ || NULL == calibration_map_left_2_)
                    ROS_ERROR("Camera: undistort maps not initialized");
                else {

                    const CvScalar outlierColor = {{0.0}};

                    left_rgb_rect_image_.data.resize(imageSize);

                    IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    sourceImageP->imageData = reinterpret_cast<char*>(&(left_rgb_image_.data[0]));
                    IplImage *destImageP    = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    destImageP->imageData   = reinterpret_cast<char*>(&(left_rgb_rect_image_.data[0]));

                    cvRemap(sourceImageP, destImageP, 
                            calibration_map_left_1_, 
                            calibration_map_left_2_,
                            CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, 
                            outlierColor);

                    cvReleaseImageHeader(&sourceImageP);
                    cvReleaseImageHeader(&destImageP);

                    left_rgb_rect_image_.header.frame_id = frame_id_left_;
                    left_rgb_rect_image_.header.stamp    = ros::Time(header.timeSeconds,
                                                                     1000 * header.timeMicroSeconds);
                    left_rgb_rect_image_.height          = height;
                    left_rgb_rect_image_.width           = width;
                    
                    left_rgb_rect_image_.encoding        = "rgb8";
                    left_rgb_rect_image_.is_bigendian    = false;
                    left_rgb_rect_image_.step            = 3 * width;
                    
                    left_rgb_rect_cam_info_.header = left_rgb_rect_image_.header;
                    left_rgb_rect_frame_id_              = header.frameId;
                    
                    if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0)
                        left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rgb_rect_cam_info_);
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

    //
    // Frame IDs must match for the rectified images

    left_rect_cam_info_.header.frame_id  = frame_id_left_;
    right_rect_cam_info_.header.frame_id = left_rect_cam_info_.header.frame_id;

    left_rect_cam_info_.width  = c.width();
    left_rect_cam_info_.height = c.height();
    left_rect_cam_info_.P[0]   = c.fx();       left_rect_cam_info_.P[1]   = 0.0;   
    left_rect_cam_info_.P[4]   = 0.0;          left_rect_cam_info_.P[5]   = c.fy();  
    left_rect_cam_info_.P[8]   = 0.0;          left_rect_cam_info_.P[9]   = 0.0;   
    left_rect_cam_info_.P[2]   = c.cx();       left_rect_cam_info_.P[3]   = 0.0;
    left_rect_cam_info_.P[6]   = c.cy();       left_rect_cam_info_.P[7]   = 0.0;
    left_rect_cam_info_.P[10]  = 1.0;          left_rect_cam_info_.P[11]  = 0.0;
    
    right_rect_cam_info_.width  = c.width();
    right_rect_cam_info_.height = c.height();
    right_rect_cam_info_.P[0]   = c.fx();      right_rect_cam_info_.P[1]  = 0.0;  
    right_rect_cam_info_.P[4]   = 0.0;         right_rect_cam_info_.P[5]  = c.fy();  
    right_rect_cam_info_.P[8]   = 0.0;         right_rect_cam_info_.P[9]  = 0.0;  
    right_rect_cam_info_.P[2]   = c.cx();      right_rect_cam_info_.P[3]  = c.tx() * c.fx();
    right_rect_cam_info_.P[6]   = c.cy();      right_rect_cam_info_.P[7]  = 0.0;
    right_rect_cam_info_.P[10]  = 1.0;         right_rect_cam_info_.P[11] = 0.0;
    
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
    // For local rectification of color images

    left_rgb_rect_cam_info_ = left_rect_cam_info_;
    
    if (calibration_map_left_1_)
        cvReleaseMat(&calibration_map_left_1_);
    if (calibration_map_left_2_)
        cvReleaseMat(&calibration_map_left_2_);

    calibration_map_left_1_ = cvCreateMat(c.height(), c.width(), CV_32F);
    calibration_map_left_2_ = cvCreateMat(c.height(), c.width(), CV_32F);

    //
    // Calibration from sensor is for native resolution
    
    image::Calibration cal = image_calibration_;

    const float x_scale = 1.0f / ((static_cast<float>(device_info_.imagerWidth) / 
                                   static_cast<float>(c.width())));
    const float y_scale = 1.0f / ((static_cast<float>(device_info_.imagerHeight) / 
                                   static_cast<float>(c.height())));

    cal.left.M[0][0] *= x_scale;  cal.left.M[1][1] *= y_scale;
    cal.left.M[0][2] *= x_scale;  cal.left.M[1][2] *= y_scale;
    cal.left.P[0][0] *= x_scale;  cal.left.P[1][1] *= y_scale;
    cal.left.P[0][2] *= x_scale;  cal.left.P[1][2] *= y_scale;
    cal.left.P[0][3] *= x_scale;  cal.left.P[1][3] *= y_scale;

    CvMat M1 = cvMat(3, 3, CV_32F, &cal.left.M);
    CvMat D1 = cvMat(1, 8, CV_32F, &cal.left.D);
    CvMat R1 = cvMat(3, 3, CV_32F, &cal.left.R);
    CvMat P1 = cvMat(3, 4, CV_32F, &cal.left.P);
    
    cvInitUndistortRectifyMap(&M1, &D1, &R1, &P1, 
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
    cfg.tz    = c.tx();
    cfg.roll  = c.roll();
    cfg.pitch = c.pitch();
    cfg.yaw   = c.yaw();

    raw_cam_config_pub_.publish(cfg);
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
