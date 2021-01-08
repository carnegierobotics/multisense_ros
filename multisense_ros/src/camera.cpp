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

#include <arpa/inet.h>
#include <fstream>
#include <turbojpeg.h>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <multisense_lib/MultiSenseChannel.hh>
#include <multisense_ros/camera.h>
#include <multisense_ros/RawCamConfig.h>
#include <multisense_ros/RawCamCal.h>
#include <multisense_ros/DeviceInfo.h>
#include <multisense_ros/Histogram.h>
#include <multisense_ros/point_cloud_utilities.h>

using namespace crl::multisense;

namespace multisense_ros {

namespace { // anonymous

tf2::Matrix3x3 toRotation(float R[3][3])
{
    return tf2::Matrix3x3{R[0][0],
                          R[0][1],
                          R[0][2],
                          R[1][0],
                          R[1][1],
                          R[1][2],
                          R[2][0],
                          R[2][1],
                          R[2][2]};
}

//
// All of the data sources that we control here

constexpr DataSource allImageSources = (Source_Luma_Left             |
                                        Source_Luma_Right            |
                                        Source_Luma_Aux              |
                                        Source_Luma_Rectified_Left   |
                                        Source_Luma_Rectified_Right  |
                                        Source_Luma_Rectified_Aux    |
                                        Source_Chroma_Rectified_Aux  |
                                        Source_Chroma_Left           |
                                        Source_Chroma_Aux            |
                                        Source_Disparity             |
                                        Source_Disparity_Right       |
                                        Source_Disparity_Cost        |
                                        Source_Jpeg_Left);

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
void colorizeCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorizeCallback(header); }

bool isValidReprojectedPoint(const Eigen::Vector3f& pt, double squared_max_range)
{
    return pt[2] > 0.0f && std::isfinite(pt[2]) && pt.squaredNorm() < squared_max_range;
}

void writePoint(sensor_msgs::PointCloud2 &pointcloud, size_t index, const Eigen::Vector3f &point, uint32_t color)
{
    float* cloudP = reinterpret_cast<float*>(&(pointcloud.data[index * pointcloud.point_step]));
    cloudP[0] = point[0];
    cloudP[1] = point[1];
    cloudP[2] = point[2];

    uint32_t* colorP = reinterpret_cast<uint32_t*>(&(cloudP[3]));
    colorP[0] = color;
}

void writePoint(sensor_msgs::PointCloud2 &pointcloud,
                size_t pointcloud_index,
                const Eigen::Vector3f &point,
                size_t image_index,
                const image::Header &image)
{
    switch (image.bitsPerPixel)
    {
        case 8:
        {
            const uint32_t luma = static_cast<uint32_t>(reinterpret_cast<const uint8_t*>(image.imageDataP)[image_index]);
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
        case 16:
        {
            const uint32_t luma = static_cast<uint32_t>(reinterpret_cast<const uint16_t*>(image.imageDataP)[image_index]);
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
        case 32:
        {
            const uint32_t luma = reinterpret_cast<const uint32_t*>(image.imageDataP)[image_index];
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
    }
}

bool clipPoint(const BorderClip& borderClipType,
               double borderClipValue,
               size_t height,
               size_t width,
               size_t u,
               size_t v)
{
    switch (borderClipType)
    {
        case BorderClip::NONE:
        {
            return false;
        }
        case BorderClip::RECTANGULAR:
        {
            return !( u >= borderClipValue && u <= width - borderClipValue &&
                      v >= borderClipValue && v <= height - borderClipValue);
        }
        case BorderClip::CIRCULAR:
        {
            const double halfWidth = static_cast<double>(width)/2.0;
            const double halfHeight = static_cast<double>(height)/2.0;

            const double radius = sqrt( halfWidth * halfWidth + halfHeight * halfHeight ) - borderClipValue;

            return !(Eigen::Vector2d{halfWidth - u, halfHeight - v}.norm() < radius);
        }
        default:
        {
            ROS_WARN("Camera: Unknown border clip type.");
            break;
        }
    }

    return true;
}

cv::Vec3b u_interpolate_color(double u, double v, const cv::Mat &image)
{
    const double width = image.cols;

    //
    // Interpolate in just the u dimension
    //
    const size_t min_u = static_cast<size_t>(std::min(std::max(std::floor(u), 0.), width - 1.));
    const size_t max_u = static_cast<size_t>(std::min(std::max(std::ceil(u), 0.), width - 1.));

    const cv::Vec3d element0 = image.at<cv::Vec3b>(width * v + min_u);
    const cv::Vec3d element1 = image.at<cv::Vec3b>(width * v + max_u);

    const size_t delta_u = max_u - min_u;

    const double u_ratio = delta_u == 0 ? 1. : (static_cast<double>(max_u) - u) / static_cast<double>(delta_u);

    const cv::Vec3b result = (element0 * u_ratio + element1 * (1. - u_ratio));

    return result;
}

} // anonymous

//
// Provide compiler with definition of the static members

constexpr char Camera::LEFT[];
constexpr char Camera::RIGHT[];
constexpr char Camera::AUX[];
constexpr char Camera::CALIBRATION[];

constexpr char Camera::LEFT_CAMERA_FRAME[];
constexpr char Camera::RIGHT_CAMERA_FRAME[];
constexpr char Camera::LEFT_RECTIFIED_FRAME[];
constexpr char Camera::RIGHT_RECTIFIED_FRAME[];
constexpr char Camera::AUX_CAMERA_FRAME[];
constexpr char Camera::AUX_RECTIFIED_FRAME[];

constexpr char Camera::DEVICE_INFO_TOPIC[];
constexpr char Camera::RAW_CAM_CAL_TOPIC[];
constexpr char Camera::RAW_CAM_CONFIG_TOPIC[];
constexpr char Camera::RAW_CAM_DATA_TOPIC[];
constexpr char Camera::HISTOGRAM_TOPIC[];
constexpr char Camera::MONO_TOPIC[];
constexpr char Camera::RECT_TOPIC[];
constexpr char Camera::DISPARITY_TOPIC[];
constexpr char Camera::DISPARITY_IMAGE_TOPIC[];
constexpr char Camera::DEPTH_TOPIC[];
constexpr char Camera::OPENNI_DEPTH_TOPIC[];
constexpr char Camera::COST_TOPIC[];
constexpr char Camera::COLOR_TOPIC[];
constexpr char Camera::RECT_COLOR_TOPIC[];
constexpr char Camera::POINTCLOUD_TOPIC[];
constexpr char Camera::COLOR_POINTCLOUD_TOPIC[];
constexpr char Camera::ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char Camera::COLOR_ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char Camera::MONO_CAMERA_INFO_TOPIC[];
constexpr char Camera::RECT_CAMERA_INFO_TOPIC[];
constexpr char Camera::COLOR_CAMERA_INFO_TOPIC[];
constexpr char Camera::RECT_COLOR_CAMERA_INFO_TOPIC[];
constexpr char Camera::DEPTH_CAMERA_INFO_TOPIC[];
constexpr char Camera::DISPARITY_CAMERA_INFO_TOPIC[];
constexpr char Camera::COST_CAMERA_INFO_TOPIC[];

Camera::Camera(Channel* driver, const std::string& tf_prefix) :
    driver_(driver),
    device_nh_(""),
    left_nh_(device_nh_, LEFT),
    right_nh_(device_nh_, RIGHT),
    aux_nh_(device_nh_, AUX),
    calibration_nh_(device_nh_, CALIBRATION),
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
    aux_mono_transport_(aux_nh_),
    aux_rgb_transport_(aux_nh_),
    aux_rect_transport_(aux_nh_),
    aux_rgb_rect_transport_(aux_nh_),
    frame_id_left_(tf_prefix + LEFT_CAMERA_FRAME),
    frame_id_right_(tf_prefix + RIGHT_CAMERA_FRAME),
    frame_id_aux_(tf_prefix + AUX_CAMERA_FRAME),
    frame_id_rectified_left_(tf_prefix + LEFT_RECTIFIED_FRAME),
    frame_id_rectified_right_(tf_prefix + RIGHT_RECTIFIED_FRAME),
    frame_id_rectified_aux_(tf_prefix + AUX_RECTIFIED_FRAME),
    pointcloud_max_range_(15.0),
    last_frame_id_(-1),
    border_clip_type_(BorderClip::NONE),
    border_clip_value_(0.0)
{
    //
    // Query device and version information from sensor

    Status status = driver_->getVersionInfo(version_info_);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query version info: %s", Channel::statusString(status));
        return;
    }

    status = driver_->getDeviceInfo(device_info_);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query device info: %s", Channel::statusString(status));
        return;
    }

    //
    // Get the camera config

    image::Config image_config;
    status = driver_->getImageConfig(image_config);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query sensor configuration: %s", Channel::statusString(status));
        return;
    }

    //
    // S27/S30 cameras have a 3rd aux color camera and no left color camera

    has_aux_camera_ = system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == device_info_.hardwareRevision ||
                      system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == device_info_.hardwareRevision;

    //
    // Topics published for all device types

    device_info_pub_    = calibration_nh_.advertise<multisense_ros::DeviceInfo>(DEVICE_INFO_TOPIC, 1, true);
    raw_cam_cal_pub_    = calibration_nh_.advertise<multisense_ros::RawCamCal>(RAW_CAM_CAL_TOPIC, 1, true);
    raw_cam_config_pub_ = calibration_nh_.advertise<multisense_ros::RawCamConfig>(RAW_CAM_CONFIG_TOPIC, 1, true);
    histogram_pub_      = device_nh_.advertise<multisense_ros::Histogram>(HISTOGRAM_TOPIC, 5);

    //
    // Create topic publishers (TODO: color topics should not be advertised if the device can't support it)

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        left_mono_cam_pub_  = left_mono_transport_.advertise(MONO_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Left));

        left_rgb_cam_pub_   = left_rgb_transport_.advertise(COLOR_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));

        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera(RECT_COLOR_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Jpeg_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Jpeg_Left));

        left_mono_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(MONO_CAMERA_INFO_TOPIC, 1, true);
        left_rgb_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, 1, true);
        left_rgb_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, 1, true);


    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_M == device_info_.hardwareRevision) {

        // monocular variation

        left_mono_cam_pub_  = left_mono_transport_.advertise(MONO_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        left_rect_cam_pub_  = left_rect_transport_.advertiseCamera(RECT_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left));
        left_rgb_cam_pub_   = left_rgb_transport_.advertise(COLOR_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
        left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera(RECT_COLOR_TOPIC, 5,
                                 std::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                 std::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));

        left_mono_cam_info_pub_     = left_nh_.advertise<sensor_msgs::CameraInfo>(MONO_CAMERA_INFO_TOPIC, 1, true);
        left_rect_cam_info_pub_     = left_nh_.advertise<sensor_msgs::CameraInfo>(RECT_CAMERA_INFO_TOPIC, 1, true);
        left_rgb_cam_info_pub_      = left_nh_.advertise<sensor_msgs::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, 1, true);
        left_rgb_rect_cam_info_pub_ = left_nh_.advertise<sensor_msgs::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, 1, true);

    } else {  // all other MultiSense-S* variations


        left_mono_cam_pub_  = left_mono_transport_.advertise(MONO_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Left));
        right_mono_cam_pub_ = right_mono_transport_.advertise(MONO_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Right),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Right));
        left_rect_cam_pub_  = left_rect_transport_.advertiseCamera(RECT_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left));
        right_rect_cam_pub_ = right_rect_transport_.advertiseCamera(RECT_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Right),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Right));
        depth_cam_pub_      = depth_transport_.advertise(DEPTH_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Disparity));
        ni_depth_cam_pub_   = ni_depth_transport_.advertise(OPENNI_DEPTH_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Disparity));

        if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 != device_info_.hardwareRevision) {

            if (has_aux_camera_) {

                aux_mono_cam_pub_  = aux_mono_transport_.advertise(MONO_TOPIC, 5,
                                      std::bind(&Camera::connectStream, this, Source_Luma_Aux),
                                      std::bind(&Camera::disconnectStream, this, Source_Luma_Aux));

                aux_mono_cam_info_pub_  = aux_nh_.advertise<sensor_msgs::CameraInfo>(MONO_CAMERA_INFO_TOPIC, 1, true);

                aux_rgb_cam_pub_  = aux_rgb_transport_.advertise(COLOR_TOPIC, 5,
                                      std::bind(&Camera::connectStream, this, Source_Luma_Aux | Source_Chroma_Aux),
                                      std::bind(&Camera::disconnectStream, this, Source_Luma_Aux | Source_Chroma_Aux));

                aux_rgb_cam_info_pub_  = aux_nh_.advertise<sensor_msgs::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, 1, true);

                aux_rect_cam_pub_ = aux_rect_transport_.advertiseCamera(RECT_TOPIC, 5,
                                          std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Aux),
                                          std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Aux));

                aux_rect_cam_info_pub_  = aux_nh_.advertise<sensor_msgs::CameraInfo>(RECT_CAMERA_INFO_TOPIC, 1, true);

                aux_rgb_rect_cam_pub_ = aux_rgb_rect_transport_.advertiseCamera(RECT_COLOR_TOPIC, 5,
                                          std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux),
                                          std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux));

                aux_rgb_rect_cam_info_pub_  = aux_nh_.advertise<sensor_msgs::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, 1, true);
            }
            else {

                left_rgb_cam_pub_   = left_rgb_transport_.advertise(COLOR_TOPIC, 5,
                                      std::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                      std::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));

                left_rgb_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, 1, true);
                left_rgb_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, 1, true);

                left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera(RECT_COLOR_TOPIC, 5,
                                      std::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                                      std::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));

            }

            const auto point_cloud_color_topics = has_aux_camera_ ? Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux :
                                                                   Source_Luma_Left | Source_Chroma_Left;

            color_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>(COLOR_POINTCLOUD_TOPIC, 5,
                                  std::bind(&Camera::connectStream, this,
                                  Source_Disparity | point_cloud_color_topics),
                                  std::bind(&Camera::disconnectStream, this,
                                  Source_Disparity | point_cloud_color_topics));
            color_organized_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>(COLOR_ORGANIZED_POINTCLOUD_TOPIC, 5,
                                  std::bind(&Camera::connectStream, this,
                                  Source_Disparity | point_cloud_color_topics),
                                  std::bind(&Camera::disconnectStream, this,
                                  Source_Disparity | point_cloud_color_topics));

        }

        luma_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        luma_organized_point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud2>(ORGANIZED_POINTCLOUD_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        raw_cam_data_pub_   = calibration_nh_.advertise<multisense_ros::RawCamData>(RAW_CAM_DATA_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

        left_disparity_pub_ = disparity_left_transport_.advertise(DISPARITY_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Disparity));

        left_stereo_disparity_pub_ = left_nh_.advertise<stereo_msgs::DisparityImage>(DISPARITY_IMAGE_TOPIC, 5,
                              std::bind(&Camera::connectStream, this, Source_Disparity),
                              std::bind(&Camera::disconnectStream, this, Source_Disparity));

        if (version_info_.sensorFirmwareVersion >= 0x0300) {

            right_disparity_pub_ = disparity_right_transport_.advertise(DISPARITY_TOPIC, 5,
                                   std::bind(&Camera::connectStream, this, Source_Disparity_Right),
                                   std::bind(&Camera::disconnectStream, this, Source_Disparity_Right));

            right_stereo_disparity_pub_ = right_nh_.advertise<stereo_msgs::DisparityImage>(DISPARITY_IMAGE_TOPIC, 5,
                                  std::bind(&Camera::connectStream, this, Source_Disparity_Right),
                                  std::bind(&Camera::disconnectStream, this, Source_Disparity_Right));

            left_disparity_cost_pub_ = disparity_cost_transport_.advertise(COST_TOPIC, 5,
                                   std::bind(&Camera::connectStream, this, Source_Disparity_Cost),
                                   std::bind(&Camera::disconnectStream, this, Source_Disparity_Cost));

            right_disp_cam_info_pub_  = right_nh_.advertise<sensor_msgs::CameraInfo>(DISPARITY_CAMERA_INFO_TOPIC, 1, true);
            left_cost_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(COST_CAMERA_INFO_TOPIC, 1, true);
        }

        //
        // Camera info topic publishers
        left_mono_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(MONO_CAMERA_INFO_TOPIC, 1, true);
        right_mono_cam_info_pub_ = right_nh_.advertise<sensor_msgs::CameraInfo>(MONO_CAMERA_INFO_TOPIC, 1, true);
        left_rect_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(RECT_CAMERA_INFO_TOPIC, 1, true);
        right_rect_cam_info_pub_ = right_nh_.advertise<sensor_msgs::CameraInfo>(RECT_CAMERA_INFO_TOPIC, 1, true);
        left_disp_cam_info_pub_  = left_nh_.advertise<sensor_msgs::CameraInfo>(DISPARITY_CAMERA_INFO_TOPIC, 1, true);
        depth_cam_info_pub_ = device_nh_.advertise<sensor_msgs::CameraInfo>(DEPTH_CAMERA_INFO_TOPIC, 1, true);
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
    for(const auto &pcb : device_info_.pcbs) {
        msg.pcbSerialNumbers.push_back(pcb.revision);
        msg.pcbNames.push_back(pcb.name);
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

    image::Calibration image_calibration;
    status = driver_->getImageCalibration(image_calibration);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query image calibration: %s",
                  Channel::statusString(status));
    }
    else {

        multisense_ros::RawCamCal cal;
        const float              *cP;

        cP = reinterpret_cast<const float *>(&(image_calibration.left.M[0][0]));
        for(uint32_t i=0; i<9; i++) cal.left_M[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.left.D[0]));
        for(uint32_t i=0; i<8; i++) cal.left_D[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.left.R[0][0]));
        for(uint32_t i=0; i<9; i++) cal.left_R[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.left.P[0][0]));
        for(uint32_t i=0; i<12; i++) cal.left_P[i] = cP[i];

        cP = reinterpret_cast<const float *>(&(image_calibration.right.M[0][0]));
        for(uint32_t i=0; i<9; i++) cal.right_M[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.right.D[0]));
        for(uint32_t i=0; i<8; i++) cal.right_D[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.right.R[0][0]));
        for(uint32_t i=0; i<9; i++) cal.right_R[i] = cP[i];
        cP = reinterpret_cast<const float *>(&(image_calibration.right.P[0][0]));
        for(uint32_t i=0; i<12; i++) cal.right_P[i] = cP[i];

        raw_cam_cal_pub_.publish(cal);
    }

    stereo_calibration_manager_ = std::make_shared<StereoCalibrationManger>(image_config, image_calibration, device_info_);

    if (has_aux_camera_ && !stereo_calibration_manager_->validAux()) {
        ROS_WARN("Camera: invalid aux camera calibration");
    }

    //
    // Publish the static transforms for our camera extrinsics for the left/right/aux frames. We will
    // use the left camera frame as the reference coordinate frame

    const bool has_aux_extrinsics = has_aux_camera_ && stereo_calibration_manager_->validAux();

    std::vector<geometry_msgs::TransformStamped> stamped_transforms(3 + (has_aux_extrinsics ? 2 : 0));

    tf2::Transform rectified_left_T_left{toRotation(image_calibration.left.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[0].header.stamp = ros::Time::now();
    stamped_transforms[0].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[0].child_frame_id = frame_id_left_;
    stamped_transforms[0].transform = tf2::toMsg(rectified_left_T_left);

    tf2::Transform rectified_right_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                    tf2::Vector3{stereo_calibration_manager_->T(), 0., 0.}};
    stamped_transforms[1].header.stamp = ros::Time::now();
    stamped_transforms[1].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[1].child_frame_id = frame_id_rectified_right_;
    stamped_transforms[1].transform = tf2::toMsg(rectified_right_T_rectified_left.inverse());

    tf2::Transform rectified_right_T_right{toRotation(image_calibration.right.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[2].header.stamp = ros::Time::now();
    stamped_transforms[2].header.frame_id = frame_id_rectified_right_;
    stamped_transforms[2].child_frame_id = frame_id_right_;
    stamped_transforms[2].transform = tf2::toMsg(rectified_right_T_right);

    if (has_aux_extrinsics)
    {
        tf2::Transform rectified_aux_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                      tf2::Vector3{stereo_calibration_manager_->aux_T(), 0., 0.}};
        stamped_transforms[3].header.stamp = ros::Time::now();
        stamped_transforms[3].header.frame_id = frame_id_rectified_left_;
        stamped_transforms[3].child_frame_id = frame_id_rectified_aux_;
        stamped_transforms[3].transform = tf2::toMsg(rectified_aux_T_rectified_left.inverse());

        tf2::Transform rectified_aux_T_aux{toRotation(image_calibration.aux.R), tf2::Vector3{0., 0., 0.}};
        stamped_transforms[4].header.stamp = ros::Time::now();
        stamped_transforms[4].header.frame_id = frame_id_rectified_aux_;
        stamped_transforms[4].child_frame_id = frame_id_aux_;
        stamped_transforms[4].transform = tf2::toMsg(rectified_aux_T_aux);
    }

    static_tf_broadcaster_.sendTransform(stamped_transforms);

    //
    // Update our internal image config and publish intitial camera info

    updateConfig(image_config);

    //
    // Initialize point cloud data structures

    luma_point_cloud_ = initialize_pointcloud<float>(true, frame_id_rectified_left_, "intensity");
    color_point_cloud_ = initialize_pointcloud<float>(true, frame_id_rectified_left_, "rgb");
    luma_organized_point_cloud_ = initialize_pointcloud<float>(false, frame_id_rectified_left_, "intensity");
    color_organized_point_cloud_ = initialize_pointcloud<float>(false, frame_id_rectified_left_, "rgb");

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

        driver_->addIsolatedCallback(colorizeCB, Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux | Source_Luma_Aux |
                                                 Source_Luma_Left | Source_Chroma_Left | Source_Luma_Rectified_Left, this);
        driver_->addIsolatedCallback(monoCB,  Source_Luma_Left | Source_Luma_Right | Source_Luma_Aux, this);
        driver_->addIsolatedCallback(rectCB,  Source_Luma_Rectified_Left | Source_Luma_Rectified_Right | Source_Luma_Rectified_Aux, this);
        driver_->addIsolatedCallback(depthCB, Source_Disparity, this);
        driver_->addIsolatedCallback(pointCB, Source_Disparity, this);
        driver_->addIsolatedCallback(rawCB,   Source_Disparity | Source_Luma_Rectified_Left, this);
        driver_->addIsolatedCallback(colorCB, Source_Chroma_Left | Source_Chroma_Rectified_Aux | Source_Chroma_Aux, this);
        driver_->addIsolatedCallback(dispCB,  Source_Disparity | Source_Disparity_Right | Source_Disparity_Cost, this);
    }

    //
    // A common callback to publish histograms

    driver_->addIsolatedCallback(histCB, allImageSources, this);

    //
    // Disable color point cloud strict frame syncing, if desired

    const char *pcColorFrameSyncEnvStringP = getenv("MULTISENSE_ROS_PC_COLOR_FRAME_SYNC_OFF");
    if (NULL != pcColorFrameSyncEnvStringP) {
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

        driver_->removeIsolatedCallback(colorizeCB);
        driver_->removeIsolatedCallback(monoCB);
        driver_->removeIsolatedCallback(rectCB);
        driver_->removeIsolatedCallback(depthCB);
        driver_->removeIsolatedCallback(pointCB);
        driver_->removeIsolatedCallback(rawCB);
        driver_->removeIsolatedCallback(colorCB);
        driver_->removeIsolatedCallback(dispCB);
    }
}

void Camera::borderClipChanged(const BorderClip &borderClipType, double borderClipValue)
{
    //
    // Avoid locking since updating this while a point cloud is current being projected is a non-standard case

    border_clip_type_ = borderClipType;
    border_clip_value_ = borderClipValue;
}

void Camera::maxPointCloudRangeChanged(double range)
{
    pointcloud_max_range_ = range;
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
    if (Source_Jpeg_Left != header.source) {
        return;
    }

    const ros::Time t = ros::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);

    const uint32_t height    = header.height;
    const uint32_t width     = header.width;
    const uint32_t rgbLength = height * width * 3;

    left_rgb_image_.header.frame_id = frame_id_left_;
    left_rgb_image_.height          = height;
    left_rgb_image_.width           = width;
    left_rgb_image_.encoding        = sensor_msgs::image_encodings::RGB8;
    left_rgb_image_.is_bigendian    = (htonl(1) == 1);
    left_rgb_image_.step            = 3 * width;
    left_rgb_image_.header.stamp    = t;

    left_rgb_image_.data.resize(rgbLength);

    tjhandle jpegDecompressor = tjInitDecompress();
    tjDecompress2(jpegDecompressor,
                  reinterpret_cast<unsigned char*>(const_cast<void*>(header.imageDataP)),
                  header.imageLength,
                  &(left_rgb_image_.data[0]),
                  width, 0/*pitch*/, height, TJPF_RGB, 0);
    tjDestroy(jpegDecompressor);

    const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_left_, t);

    left_rgb_cam_pub_.publish(left_rgb_image_);
    left_rgb_cam_info_pub_.publish(left_camera_info);

    if (left_rgb_rect_cam_pub_.getNumSubscribers() > 0) {

        const auto left_rectified_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);

        left_rgb_rect_image_.data.resize(rgbLength);

        const cv::Mat rgb_image(height, width, CV_8UC3, &(left_rgb_image_.data[0]));
        cv::Mat rect_rgb_image(height, width, CV_8UC3, &(left_rgb_rect_image_.data[0]));

        const auto left_remap = stereo_calibration_manager_->leftRemap();

        cv::remap(rgb_image, rect_rgb_image, left_remap->map1, left_remap->map2, cv::INTER_LINEAR);

        left_rgb_rect_image_.header.frame_id = frame_id_rectified_left_;
        left_rgb_rect_image_.header.stamp    = t;
        left_rgb_rect_image_.height          = height;
        left_rgb_rect_image_.width           = width;
        left_rgb_rect_image_.encoding        = sensor_msgs::image_encodings::RGB8;
        left_rgb_rect_image_.is_bigendian    = (htonl(1) == 1);
        left_rgb_rect_image_.step            = 3 * width;
        left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rectified_camera_info);
        left_rgb_rect_cam_info_pub_.publish(left_rectified_camera_info);
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

    const ros::Time t = ros::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Disparity:
    case Source_Disparity_Right:
    {
        sensor_msgs::Image         *imageP   = NULL;
        image_transport::Publisher *pubP     = NULL;
        sensor_msgs::CameraInfo camInfo;
        ros::Publisher *camInfoPubP          = NULL;
        ros::Publisher *stereoDisparityPubP  = NULL;
        stereo_msgs::DisparityImage *stereoDisparityImageP = NULL;


        if (Source_Disparity == header.source) {
            pubP                    = &left_disparity_pub_;
            imageP                  = &left_disparity_image_;
            imageP->header.frame_id = frame_id_rectified_left_;
            camInfo                 = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);
            camInfoPubP             = &left_disp_cam_info_pub_;
            stereoDisparityPubP     = &left_stereo_disparity_pub_;
            stereoDisparityImageP   = &left_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_rectified_left_;
        } else {
            pubP                    = &right_disparity_pub_;
            imageP                  = &right_disparity_image_;
            imageP->header.frame_id = frame_id_rectified_right_;
            camInfo                 = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t);
            camInfoPubP             = &right_disp_cam_info_pub_;
            stereoDisparityPubP     = &right_stereo_disparity_pub_;
            stereoDisparityImageP   = &right_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_rectified_right_;
        }

        if (pubP->getNumSubscribers() > 0)
        {
            imageP->data.resize(imageSize);
            memcpy(&imageP->data[0], header.imageDataP, imageSize);

            imageP->header.stamp    = t;
            imageP->height          = header.height;
            imageP->width           = header.width;
            imageP->is_bigendian    = (htonl(1) == 1);

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

            if (camInfo.P[0] != camInfo.P[5])
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
            stereoDisparityImageP->image.is_bigendian = (htonl(1) == 1);
            stereoDisparityImageP->image.header.stamp = t;
            stereoDisparityImageP->image.header.frame_id = stereoDisparityImageP->header.frame_id;
            stereoDisparityImageP->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            stereoDisparityImageP->image.step = 4 * header.width;


            //
            // Fx is the same for both the right and left cameras

            stereoDisparityImageP->f = camInfo.P[0];

            //
            // Our Tx is negative. The DisparityImage message expects Tx to be
            // positive

            stereoDisparityImageP->T = fabs(stereo_calibration_manager_->T());
            stereoDisparityImageP->min_disparity = 0;
            stereoDisparityImageP->max_disparity = stereo_calibration_manager_->config().disparities();
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

        camInfoPubP->publish(camInfo);

        break;
    }
    case Source_Disparity_Cost:

        left_disparity_cost_image_.data.resize(imageSize);
        memcpy(&left_disparity_cost_image_.data[0], header.imageDataP, imageSize);

        left_disparity_cost_image_.header.frame_id = frame_id_rectified_left_;
        left_disparity_cost_image_.header.stamp    = t;
        left_disparity_cost_image_.height          = header.height;
        left_disparity_cost_image_.width           = header.width;

        left_disparity_cost_image_.encoding        = sensor_msgs::image_encodings::MONO8;
        left_disparity_cost_image_.is_bigendian    = (htonl(1) == 1);
        left_disparity_cost_image_.step            = header.width;

        left_disparity_cost_pub_.publish(left_disparity_cost_image_);

        left_cost_cam_info_pub_.publish(stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t));

        break;
    }
}

void Camera::monoCallback(const image::Header& header)
{
    if (Source_Luma_Left  != header.source &&
        Source_Luma_Right != header.source &&
        Source_Luma_Aux != header.source) {

        ROS_ERROR("Camera: unexpected mono image source: 0x%x", header.source);
        return;
    }

    ros::Time t = ros::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Left:
    {

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

        left_mono_image_.is_bigendian    = (htonl(1) == 1);

        left_mono_cam_pub_.publish(left_mono_image_);

        //
        // Publish a specific camera info message for the left mono image
        left_mono_cam_info_pub_.publish(stereo_calibration_manager_->leftCameraInfo(frame_id_left_, t));

        break;
    }
    case Source_Luma_Right:
    {
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
        right_mono_image_.is_bigendian    = (htonl(1) == 1);

        right_mono_cam_pub_.publish(right_mono_image_);

        //
        // Publish a specific camera info message for the right mono image
        right_mono_cam_info_pub_.publish(stereo_calibration_manager_->rightCameraInfo(frame_id_right_, t));

        break;
    }
    case Source_Luma_Aux:
    {
        aux_mono_image_.data.resize(header.imageLength);
        memcpy(&aux_mono_image_.data[0], header.imageDataP, header.imageLength);

        aux_mono_image_.header.frame_id = frame_id_aux_;
        aux_mono_image_.header.stamp    = t;
        aux_mono_image_.height          = header.height;
        aux_mono_image_.width           = header.width;

        switch(header.bitsPerPixel) {
            case 8:
                aux_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                aux_mono_image_.step     = header.width;
                break;
            case 16:
                aux_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                aux_mono_image_.step     = header.width * 2;
                break;
        }
        aux_mono_image_.is_bigendian    = (htonl(1) == 1);

        aux_mono_cam_pub_.publish(aux_mono_image_);

        //
        // Publish a specific camera info message for the aux mono image
        aux_mono_cam_info_pub_.publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, t));
        break;

    }
    }
}

void Camera::rectCallback(const image::Header& header)
{
    if (Source_Luma_Rectified_Left  != header.source &&
        Source_Luma_Rectified_Right != header.source &&
        Source_Luma_Rectified_Aux != header.source) {

        ROS_ERROR("Camera: unexpected rectified image source: 0x%x", header.source);
        return;
    }

    ros::Time t = ros::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Luma_Rectified_Left:
    {

        left_rect_image_.data.resize(header.imageLength);
        memcpy(&left_rect_image_.data[0], header.imageDataP, header.imageLength);

        left_rect_image_.header.frame_id = frame_id_rectified_left_;
        left_rect_image_.header.stamp    = t;
        left_rect_image_.height          = header.height;
        left_rect_image_.width           = header.width;

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

        left_rect_image_.is_bigendian = (htonl(1) == 1);

        const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);

        //
        // Continue to publish the rect camera info on the
        // <namespace>/left/camera_info topic for backward compatibility with
        // older versions of the driver
        left_rect_cam_pub_.publish(left_rect_image_, left_camera_info);

        left_rect_cam_info_pub_.publish(left_camera_info);

        break;
    }
    case Source_Luma_Rectified_Right:
    {

        right_rect_image_.data.resize(header.imageLength);
        memcpy(&right_rect_image_.data[0], header.imageDataP, header.imageLength);

        right_rect_image_.header.frame_id = frame_id_rectified_right_;
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

        right_rect_image_.is_bigendian = (htonl(1) == 1);

        const auto right_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t);

        //
        // Continue to publish the rect camera info on the
        // <namespace>/right/camera_info topic for backward compatibility with
        // older versions of the driver
        right_rect_cam_pub_.publish(right_rect_image_, right_camera_info);

        right_rect_cam_info_pub_.publish(right_camera_info);

        break;
    }
    case Source_Luma_Rectified_Aux:
    {

        aux_rect_image_.data.resize(header.imageLength);
        memcpy(&aux_rect_image_.data[0], header.imageDataP, header.imageLength);

        aux_rect_image_.header.frame_id = frame_id_rectified_aux_;
        aux_rect_image_.header.stamp    = t;
        aux_rect_image_.height          = header.height;
        aux_rect_image_.width           = header.width;

        switch(header.bitsPerPixel) {
            case 8:
                aux_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                aux_rect_image_.step     = header.width;
                break;
            case 16:
                aux_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                aux_rect_image_.step     = header.width * 2;
                break;
        }

        aux_rect_image_.is_bigendian = (htonl(1) == 1);

        const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, t);

        //
        // Continue to publish the rect camera info on the
        // <namespace>/aux/camera_info topic for backward compatibility with
        // older versions of the driver
        aux_rect_cam_pub_.publish(aux_rect_image_, aux_camera_info);

        aux_rect_cam_info_pub_.publish(aux_camera_info);

        break;
    }
    }
}

void Camera::depthCallback(const image::Header& header)
{
    if (Source_Disparity != header.source) {

        ROS_ERROR("Camera: unexpected depth image source: 0x%x", header.source);
        return;
    }

    uint32_t niDepthSubscribers = ni_depth_cam_pub_.getNumSubscribers();
    uint32_t depthSubscribers = depth_cam_pub_.getNumSubscribers();

    if (0 == niDepthSubscribers && 0 == depthSubscribers)
    {
        return;
    }

    const ros::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t niDepthSize = header.height * header.width * sizeof(uint16_t);
    const uint32_t imageSize = header.width * header.height;

    depth_image_.header.stamp    = t;
    depth_image_.header.frame_id = frame_id_rectified_left_;
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

    const uint16_t min_ni_depth = std::numeric_limits<uint16_t>::lowest();
    const uint16_t max_ni_depth = std::numeric_limits<uint16_t>::max();

    //
    // Disparity is in 32-bit floating point

    if (32 == header.bitsPerPixel) {

        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous Cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d).
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.

        const double scale = stereo_calibration_manager_->rightCameraInfo(frame_id_right_, t).P[3];

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
                niDepthImageP[i] = static_cast<uint16_t>(std::min(static_cast<float>(max_ni_depth),
                                                                  std::max(static_cast<float>(min_ni_depth),
                                                                           depthImageP[i] * 1000)));
            }
        }

    //
    // Disparity is in 1/16th pixel, unsigned integer

    } else if (16 == header.bitsPerPixel) {

        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous Cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d). Because our disparity
        // image is 16 bits we must also divide by 16 making z = (fx*Tx*16)/(-d)
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.


        const float scale = stereo_calibration_manager_->rightCameraInfo(frame_id_right_, t).P[3] * -16.0f;

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
                niDepthImageP[i] = static_cast<uint16_t>(std::min(static_cast<float>(max_ni_depth),
                                                                  std::max(static_cast<float>(min_ni_depth),
                                                                           depthImageP[i] * 1000)));
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

    depth_cam_info_pub_.publish(stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t));
}

void Camera::pointCloudCallback(const image::Header& header)
{
    if (Source_Disparity != header.source) {

        ROS_ERROR("Camera: unexpected pointcloud image source: 0x%x", header.source);
        return;
    }

    //
    // Get the corresponding visual images so we can colorize properly

    std::shared_ptr<BufferWrapper<image::Header>> left_luma_rect = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> left_luma = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> left_chroma = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> aux_luma_rectified = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> aux_chroma_rectified = nullptr;

    const auto left_rect_image = image_buffers_.find(Source_Luma_Rectified_Left);
    if (left_rect_image != std::end(image_buffers_) && left_rect_image->second->data().frameId == header.frameId) {
        left_luma_rect = left_rect_image->second;
    }

    const auto left_luma_image = image_buffers_.find(Source_Luma_Left);
    if (left_luma_image != std::end(image_buffers_) && left_luma_image->second->data().frameId == header.frameId) {
        left_luma = left_luma_image->second;
    }

    const auto chroma_image = image_buffers_.find(Source_Chroma_Left);
    if (chroma_image != std::end(image_buffers_) && chroma_image->second->data().frameId == header.frameId) {
        left_chroma = chroma_image->second;
    }

    const auto aux_luma_rectified_image = image_buffers_.find(Source_Luma_Rectified_Aux);
    if (aux_luma_rectified_image != std::end(image_buffers_) && aux_luma_rectified_image->second->data().frameId == header.frameId) {
        aux_luma_rectified = aux_luma_rectified_image->second;
    }

    const auto aux_chroma_rectified_image = image_buffers_.find(Source_Chroma_Rectified_Aux);
    if (aux_chroma_rectified_image != std::end(image_buffers_) && aux_chroma_rectified_image->second->data().frameId == header.frameId) {
        aux_chroma_rectified = aux_chroma_rectified_image->second;
    }

    const bool color_data = (has_aux_camera_ && aux_luma_rectified && aux_chroma_rectified && stereo_calibration_manager_->validAux()) ||
                            (!has_aux_camera_ && left_luma && left_chroma);

    const bool pub_pointcloud = luma_point_cloud_pub_.getNumSubscribers() > 0 && left_luma_rect;
    const bool pub_color_pointcloud = color_point_cloud_pub_.getNumSubscribers() > 0 && color_data;
    const bool pub_organized_pointcloud = luma_organized_point_cloud_pub_.getNumSubscribers() > 0 && left_luma_rect;
    const bool pub_color_organized_pointcloud = color_organized_point_cloud_pub_.getNumSubscribers() > 0 && color_data;

    if (!(pub_pointcloud || pub_color_pointcloud || pub_organized_pointcloud || pub_color_organized_pointcloud))
    {
        return;
    }

    const ros::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    //
    // Resize our corresponding pointclouds if we plan on publishing them

    if (pub_pointcloud)
    {
        luma_point_cloud_.header.stamp = t;
        luma_point_cloud_.data.resize(header.width * header.height * luma_point_cloud_.point_step);
    }

    if (pub_color_pointcloud)
    {
        color_point_cloud_.header.stamp = t;
        color_point_cloud_.data.resize(header.width * header.height * color_point_cloud_.point_step);
    }

    if (pub_organized_pointcloud)
    {
        luma_organized_point_cloud_.header.stamp = t;
        luma_organized_point_cloud_.data.resize(header.width * header.height * luma_organized_point_cloud_.point_step);
        luma_organized_point_cloud_.width = header.width;
        luma_organized_point_cloud_.height = header.height;
        luma_organized_point_cloud_.row_step = header.width * luma_organized_point_cloud_.point_step;
    }

    if (pub_color_organized_pointcloud)
    {
        color_organized_point_cloud_.header.stamp = t;
        color_organized_point_cloud_.data.resize(header.width * header.height * color_organized_point_cloud_.point_step);
        color_organized_point_cloud_.width = header.width;
        color_organized_point_cloud_.height = header.height;
        color_organized_point_cloud_.row_step = header.width * color_organized_point_cloud_.point_step;
    }

    const Eigen::Matrix4d Q = stereo_calibration_manager_->Q();

    const Eigen::Vector3f invalid_point(std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN());

    //
    // Create rectified color image upfront if we are planning to publish color pointclouds

    cv::Mat rectified_color;
    if (!has_aux_camera_ && (pub_color_pointcloud || pub_color_organized_pointcloud))
    {
        const auto &luma = left_luma->data();

        pointcloud_color_buffer_.resize(3 * luma.width * luma.height);
        pointcloud_rect_color_buffer_.resize(3 * luma.width * luma.height);
        ycbcrToBgr(luma, left_chroma->data(), &(pointcloud_color_buffer_[0]));

        cv::Mat rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_color_buffer_[0]));
        cv::Mat rect_rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_rect_color_buffer_[0]));

        const auto left_remap = stereo_calibration_manager_->leftRemap();

        cv::remap(rgb_image, rect_rgb_image, left_remap->map1, left_remap->map2, cv::INTER_LINEAR);

        rectified_color = std::move(rect_rgb_image);
    }
    else if(has_aux_camera_ && (pub_color_pointcloud || pub_color_organized_pointcloud))
    {
        const auto &luma = aux_luma_rectified->data();

        pointcloud_rect_color_buffer_.resize(3 * luma.width * luma.height);

        ycbcrToBgr(luma, aux_chroma_rectified->data(), reinterpret_cast<uint8_t*>(&(pointcloud_rect_color_buffer_[0])));

        cv::Mat rect_rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_rect_color_buffer_[0]));

        rectified_color = std::move(rect_rgb_image);
    }

    //
    // Iterate through our disparity image once populating our pointcloud structures if we plan to publish them

    uint32_t packed_color = 0;

    const double squared_max_range = pointcloud_max_range_ * pointcloud_max_range_;

    const double aux_T = has_aux_camera_ ? stereo_calibration_manager_->aux_T() : stereo_calibration_manager_->T();
    const double T = stereo_calibration_manager_->T();

    size_t valid_points = 0;
    for (size_t y = 0 ; y < header.height ; ++y)
    {
        for (size_t x = 0 ; x < header.width ; ++x)
        {
            const size_t index = y * header.width + x;

            double disparity = 0.0f;
            switch(header.bitsPerPixel)
            {
                case 16:
                {
                    disparity = static_cast<double>(reinterpret_cast<const uint16_t*>(header.imageDataP)[index]) / 16.0f;
                    break;
                }
                case 32:
                {
                    disparity = static_cast<double>(reinterpret_cast<const float*>(header.imageDataP)[index]);
                    break;
                }
                default:
                {
                    ROS_ERROR("Camera: unsupported disparity detph: %d", header.bitsPerPixel);
                    return;
                }
            }

            //
            // We have a valid rectified color image meaning we plan to publish color pointcloud topics. Assemble the
            // color pixel here since it may be needed in our organized pointclouds

            if (!rectified_color.empty())
            {
                packed_color = 0;

                const double color_d = has_aux_camera_ ? (disparity * aux_T) / T : 0.0;

                const auto color_pixel = has_aux_camera_ ? u_interpolate_color(std::max(x - color_d, 0.), y, rectified_color) :
                                                          rectified_color.at<cv::Vec3b>(y, x);

                packed_color |= color_pixel[2] << 16 | color_pixel[1] << 8 | color_pixel[0];
            }

            //
            // If our disparity is 0 pixels our corresponding 3D point is infinite. If we plan to publish organized
            // pointclouds we will need to add a invalid point to our pointcloud(s)

            if (disparity == 0.0f || clipPoint(border_clip_type_, border_clip_value_, header.width, header.height, x, y))
            {
                if (pub_organized_pointcloud)
                {
                    writePoint(luma_organized_point_cloud_, index, invalid_point, index, left_luma_rect->data());
                }

                if (pub_color_organized_pointcloud)
                {
                    writePoint(color_organized_point_cloud_, index, invalid_point, packed_color);
                }

                continue;
            }

            const Eigen::Vector3f point = ((Q * Eigen::Vector4d(static_cast<double>(x),
                                                                static_cast<double>(y),
                                                                disparity,
                                                                1.0)).hnormalized()).cast<float>();


            const bool valid = isValidReprojectedPoint(point, squared_max_range);

            if (pub_pointcloud && valid)
            {
                writePoint(luma_point_cloud_, valid_points, point, index, left_luma_rect->data());
            }

            if(pub_color_pointcloud && valid)
            {
                writePoint(color_point_cloud_, valid_points, point, packed_color);
            }

            if (pub_organized_pointcloud)
            {
                writePoint(luma_organized_point_cloud_, index, valid ? point : invalid_point, index, left_luma_rect->data());
            }

            if (pub_color_organized_pointcloud)
            {
                writePoint(color_organized_point_cloud_, index, valid ? point : invalid_point, packed_color);
            }

            if (valid)
            {
                ++valid_points;
            }
        }
    }

    if (pub_pointcloud)
    {
        luma_point_cloud_.height = 1;
        luma_point_cloud_.row_step = valid_points * luma_point_cloud_.point_step;
        luma_point_cloud_.width = valid_points;
        luma_point_cloud_.data.resize(valid_points * luma_point_cloud_.point_step);
        luma_point_cloud_pub_.publish(luma_point_cloud_);
    }

    if(pub_color_pointcloud)
    {
        color_point_cloud_.height = 1;
        color_point_cloud_.row_step = valid_points * color_point_cloud_.point_step;
        color_point_cloud_.width = valid_points;
        color_point_cloud_.data.resize(valid_points * color_point_cloud_.point_step);
        color_point_cloud_pub_.publish(color_point_cloud_);
    }

    if (pub_organized_pointcloud)
    {
        luma_organized_point_cloud_pub_.publish(luma_organized_point_cloud_);
    }

    if (pub_color_organized_pointcloud)
    {
        color_organized_point_cloud_pub_.publish(color_organized_point_cloud_);
    }

}

void Camera::rawCamDataCallback(const image::Header& header)
{
    if (0 == raw_cam_data_pub_.getNumSubscribers()) {
        return;
    }

    if(Source_Disparity == header.source)
    {
        const auto image = image_buffers_.find(Source_Luma_Rectified_Left);
        if (image != std::end(image_buffers_) && image->second->data().frameId == header.frameId)
        {
            const auto luma_ptr = image->second;
            const auto &left_luma_rect = luma_ptr->data();

            const uint32_t left_luma_image_size = left_luma_rect.width * left_luma_rect.height;

            raw_cam_data_.gray_scale_image.resize(left_luma_image_size);
            memcpy(&(raw_cam_data_.gray_scale_image[0]),
                   left_luma_rect.imageDataP,
                   left_luma_image_size * sizeof(uint8_t));

            raw_cam_data_.frames_per_second = left_luma_rect.framesPerSecond;
            raw_cam_data_.gain              = left_luma_rect.gain;
            raw_cam_data_.exposure_time     = left_luma_rect.exposure;
            raw_cam_data_.frame_count       = left_luma_rect.frameId;
            raw_cam_data_.time_stamp        = ros::Time(left_luma_rect.timeSeconds, 1000 * left_luma_rect.timeMicroSeconds);
            raw_cam_data_.width             = left_luma_rect.width;
            raw_cam_data_.height            = left_luma_rect.height;

            const uint32_t disparity_size = header.width * header.height;

            raw_cam_data_.disparity_image.resize(disparity_size);
            memcpy(&(raw_cam_data_.disparity_image[0]),
                   header.imageDataP,
                   disparity_size * header.bitsPerPixel == 16 ? sizeof(uint16_t) : sizeof(uint32_t));

            raw_cam_data_pub_.publish(raw_cam_data_);
        }
    }
}

void Camera::colorImageCallback(const image::Header& header)
{
    //
    // The left-luma image is currently published before the matching chroma image so this can just trigger on that

    if (Source_Chroma_Left != header.source &&
        Source_Chroma_Rectified_Aux != header.source &&
        Source_Chroma_Aux != header.source)
    {
        ROS_WARN("Camera: unexpected color image source: 0x%x", header.source);
        return;
    }

    const ros::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch (header.source)
    {
    case Source_Chroma_Left:
    {

        const auto color_subscribers = left_rgb_cam_pub_.getNumSubscribers();
        const auto color_rect_subscribers = left_rgb_rect_cam_pub_.getNumSubscribers();

        if (color_subscribers == 0 && color_rect_subscribers == 0)
        {
            return;
        }

        const auto left_luma = image_buffers_.find(Source_Luma_Left);
        if (left_luma == std::end(image_buffers_)) {
            return;
        }

        const auto luma_ptr = left_luma->second;

        if (header.frameId == luma_ptr->data().frameId) {

            const uint32_t height    = luma_ptr->data().height;
            const uint32_t width     = luma_ptr->data().width;
            const uint32_t imageSize = 3 * height * width;

            left_rgb_image_.data.resize(imageSize);

            left_rgb_image_.header.frame_id = frame_id_left_;
            left_rgb_image_.header.stamp    = t;
            left_rgb_image_.height          = height;
            left_rgb_image_.width           = width;

            left_rgb_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            left_rgb_image_.is_bigendian    = (htonl(1) == 1);
            left_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0])));

            const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_left_, t);

            if (color_subscribers != 0) {
                left_rgb_cam_pub_.publish(left_rgb_image_);

                left_rgb_cam_info_pub_.publish(left_camera_info);
            }

            if (color_rect_subscribers > 0) {
                left_rgb_rect_image_.data.resize(imageSize);

                const auto left_rectified_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);

                const auto remaps = stereo_calibration_manager_->leftRemap();

                const cv::Mat rgb_image(height, width, CV_8UC3, &(left_rgb_image_.data[0]));
                cv::Mat rect_rgb_image(height, width, CV_8UC3, &(left_rgb_rect_image_.data[0]));

                cv::remap(rgb_image, rect_rgb_image, remaps->map1, remaps->map2, cv::INTER_LINEAR);

                left_rgb_rect_image_.header.frame_id = frame_id_rectified_left_;
                left_rgb_rect_image_.header.stamp    = t;
                left_rgb_rect_image_.height          = height;
                left_rgb_rect_image_.width           = width;

                left_rgb_rect_image_.encoding        = sensor_msgs::image_encodings::BGR8;
                left_rgb_rect_image_.is_bigendian    = (htonl(1) == 1);
                left_rgb_rect_image_.step            = 3 * width;

                left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rectified_camera_info);

                left_rgb_rect_cam_info_pub_.publish(left_rectified_camera_info);
            }
        }

        break;
    }
    case Source_Chroma_Rectified_Aux:
    {
        if (aux_rgb_rect_cam_pub_.getNumSubscribers() == 0) {
            return;
        }

        const auto aux_luma = image_buffers_.find(Source_Luma_Rectified_Aux);
        if (aux_luma == std::end(image_buffers_)) {
            return;
        }

        const auto luma_ptr = aux_luma->second;

        if (header.frameId == luma_ptr->data().frameId) {

            const uint32_t height    = luma_ptr->data().height;
            const uint32_t width     = luma_ptr->data().width;
            const uint32_t imageSize = 3 * height * width;

            aux_rgb_rect_image_.data.resize(imageSize);

            aux_rgb_rect_image_.header.frame_id = frame_id_rectified_aux_;
            aux_rgb_rect_image_.header.stamp    = t;
            aux_rgb_rect_image_.height          = height;
            aux_rgb_rect_image_.width           = width;

            aux_rgb_rect_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            aux_rgb_rect_image_.is_bigendian    = (htonl(1) == 1);
            aux_rgb_rect_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(aux_rgb_rect_image_.data[0])));

            const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, t);

            aux_rgb_rect_cam_pub_.publish(aux_rgb_rect_image_, aux_camera_info);

            aux_rgb_rect_cam_info_pub_.publish(aux_camera_info);
        }

        break;
    }
    case Source_Chroma_Aux:
    {
        if (aux_rgb_cam_pub_.getNumSubscribers() == 0) {
            return;
        }

        const auto aux_luma = image_buffers_.find(Source_Luma_Aux);
        if (aux_luma == std::end(image_buffers_)) {
            return;
        }

        const auto luma_ptr = aux_luma->second;

        if (header.frameId == luma_ptr->data().frameId) {
            const uint32_t height    = luma_ptr->data().height;
            const uint32_t width     = luma_ptr->data().width;
            const uint32_t imageSize = 3 * height * width;

            aux_rgb_image_.data.resize(imageSize);

            aux_rgb_image_.header.frame_id = frame_id_aux_;
            aux_rgb_image_.header.stamp    = t;
            aux_rgb_image_.height          = height;
            aux_rgb_image_.width           = width;

            aux_rgb_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            aux_rgb_image_.is_bigendian    = (htonl(1) == 1);
            aux_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(aux_rgb_image_.data[0])));

            const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, t);

            aux_rgb_cam_pub_.publish(aux_rgb_image_);

            aux_rgb_cam_info_pub_.publish(aux_camera_info);

            break;
        }
    }
    }
}

void Camera::colorizeCallback(const image::Header& header)
{
    if (Source_Luma_Rectified_Aux != header.source &&
        Source_Chroma_Rectified_Aux != header.source &&
        Source_Luma_Aux != header.source &&
        Source_Luma_Left != header.source &&
        Source_Chroma_Left != header.source &&
        Source_Luma_Rectified_Left != header.source) {
        ROS_WARN("Camera: unexpected colorized image source: 0x%x", header.source);
        return;
    }

    image_buffers_[header.source] = std::make_shared<BufferWrapper<crl::multisense::image::Header>>(driver_, header);
}


void Camera::updateConfig(const image::Config& config)
{
    stereo_calibration_manager_->updateConfig(config);

    //
    // Publish the "raw" config message

    multisense_ros::RawCamConfig cfg;

    cfg.width             = config.width();
    cfg.height            = config.height();
    cfg.frames_per_second = config.fps();
    cfg.gain              = config.gain();
    cfg.exposure_time     = config.exposure();

    cfg.fx    = config.fx();
    cfg.fy    = config.fy();
    cfg.cx    = config.cx();
    cfg.cy    = config.cy();
    cfg.tx    = config.tx();
    cfg.ty    = config.ty();
    cfg.tz    = config.tz();
    cfg.roll  = config.roll();
    cfg.pitch = config.pitch();
    cfg.yaw   = config.yaw();

    raw_cam_config_pub_.publish(cfg);

    //
    // Republish our camera info topics since the resolution changed

    publishAllCameraInfo();
}

void Camera::publishAllCameraInfo()
{
    const auto stamp = ros::Time::now();

    const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_left_, stamp);
    const auto right_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_right_, stamp);

    const auto left_rectified_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, stamp);
    const auto right_rectified_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, stamp);

    //
    // Republish camera info messages outside of image callbacks.
    // The camera info publishers are latching so the messages
    // will persist until a new message is published in one of the image
    // callbacks. This makes it easier when a user is trying access a camera_info
    // for a topic which they are not subscribed to

    if (system::DeviceInfo::HARDWARE_REV_BCAM == device_info_.hardwareRevision) {

        left_mono_cam_info_pub_.publish(left_camera_info);
        left_rgb_cam_info_pub_.publish(left_camera_info);
        left_rgb_rect_cam_info_pub_.publish(left_rectified_camera_info);

    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_M == device_info_.hardwareRevision) {

        left_mono_cam_info_pub_.publish(left_camera_info);
        left_rect_cam_info_pub_.publish(left_rectified_camera_info);
        left_rgb_cam_info_pub_.publish(left_camera_info);
        left_rgb_rect_cam_info_pub_.publish(left_rectified_camera_info);

    } else {  // all other MultiSense-S* variations

        if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 != device_info_.hardwareRevision) {

            left_rgb_cam_info_pub_.publish(left_camera_info);
            left_rgb_rect_cam_info_pub_.publish(left_rectified_camera_info);
        }

        if (version_info_.sensorFirmwareVersion >= 0x0300) {

            right_disp_cam_info_pub_.publish(right_rectified_camera_info);
            left_cost_cam_info_pub_.publish(left_rectified_camera_info);
        }

        left_mono_cam_info_pub_.publish(left_camera_info);
        left_rect_cam_info_pub_.publish(left_rectified_camera_info);
        right_mono_cam_info_pub_.publish(right_camera_info);
        right_rect_cam_info_pub_.publish(right_rectified_camera_info);
        left_disp_cam_info_pub_.publish(left_rectified_camera_info);
        depth_cam_info_pub_.publish(left_rectified_camera_info);

        if (has_aux_camera_) {

            aux_mono_cam_info_pub_.publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, stamp));
            aux_rect_cam_info_pub_.publish(stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, stamp));

            aux_rgb_cam_info_pub_.publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, stamp));
            aux_rgb_rect_cam_info_pub_.publish(stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, stamp));
        }
    }
}

void Camera::stop()
{
    std::lock_guard<std::mutex> lock(stream_lock_);

    stream_map_.clear();

    Status status = driver_->stopStreams(allImageSources);
    if (Status_Ok != status)
        ROS_ERROR("Camera: failed to stop all streams: %s",
                  Channel::statusString(status));
}

void Camera::connectStream(DataSource enableMask)
{
    std::lock_guard<std::mutex> lock(stream_lock_);

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
    std::lock_guard<std::mutex> lock(stream_lock_);

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
