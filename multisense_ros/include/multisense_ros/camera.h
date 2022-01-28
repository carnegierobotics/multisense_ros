/**
 * @file camera.h
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

#ifndef MULTISENSE_ROS_CAMERA_H
#define MULTISENSE_ROS_CAMERA_H

#include <memory>
#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <sensor_msgs/distortion_models.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <multisense_lib/MultiSenseChannel.hh>
#include <multisense_ros/RawCamData.h>
#include <multisense_ros/camera_utilities.h>
#include <multisense_ros/ground_surface_utilities.h>

namespace multisense_ros {

class Camera {
public:
    Camera(crl::multisense::Channel* driver,
           const std::string& tf_prefix);
    ~Camera();

    void updateConfig(const crl::multisense::image::Config& config);

    void monoCallback(const crl::multisense::image::Header& header);
    void rectCallback(const crl::multisense::image::Header& header);
    void depthCallback(const crl::multisense::image::Header& header);
    void pointCloudCallback(const crl::multisense::image::Header& header);
    void rawCamDataCallback(const crl::multisense::image::Header& header);
    void colorImageCallback(const crl::multisense::image::Header& header);
    void disparityImageCallback(const crl::multisense::image::Header& header);
    void jpegImageCallback(const crl::multisense::image::Header& header);
    void histogramCallback(const crl::multisense::image::Header& header);
    void colorizeCallback(const crl::multisense::image::Header& header);
    void groundSurfaceCallback(const crl::multisense::image::Header& header);
    void groundSurfaceSplineCallback(const crl::multisense::ground_surface::Header& header);

    void borderClipChanged(const BorderClip &borderClipType, double borderClipValue);

    void maxPointCloudRangeChanged(double range);

    void extrinsicsChanged(crl::multisense::system::ExternalCalibration extrinsics);

    void groundSurfaceSplineDrawParametersChanged(
        const ground_surface_utilities::SplineDrawParameters &spline_draw_params);

private:
    //
    // Node names

    static constexpr char LEFT[] = "left";
    static constexpr char RIGHT[] = "right";
    static constexpr char AUX[] = "aux";
    static constexpr char CALIBRATION[] = "calibration";
    static constexpr char GROUND_SURFACE[] = "ground_surface";

    //
    // Frames

    static constexpr char ORIGIN_FRAME[] = "/origin";
    static constexpr char HEAD_FRAME[] = "/head";
    static constexpr char LEFT_CAMERA_FRAME[] = "/left_camera_frame";
    static constexpr char LEFT_RECTIFIED_FRAME[] = "/left_camera_optical_frame";
    static constexpr char RIGHT_CAMERA_FRAME[] = "/right_camera_frame";
    static constexpr char RIGHT_RECTIFIED_FRAME[] = "/right_camera_optical_frame";
    static constexpr char AUX_CAMERA_FRAME[] = "/aux_camera_frame";
    static constexpr char AUX_RECTIFIED_FRAME[] = "/aux_camera_optical_frame";

    //
    // Topic names

    static constexpr char DEVICE_INFO_TOPIC[] = "device_info";
    static constexpr char RAW_CAM_CAL_TOPIC[] = "raw_cam_cal";
    static constexpr char RAW_CAM_CONFIG_TOPIC[] = "raw_cam_config";
    static constexpr char RAW_CAM_DATA_TOPIC[] = "raw_cam_data";
    static constexpr char HISTOGRAM_TOPIC[] = "histogram";
    static constexpr char MONO_TOPIC[] = "image_mono";
    static constexpr char RECT_TOPIC[] = "image_rect";
    static constexpr char DISPARITY_TOPIC[] = "disparity";
    static constexpr char DISPARITY_IMAGE_TOPIC[] = "disparity_image";
    static constexpr char DEPTH_TOPIC[] = "depth";
    static constexpr char OPENNI_DEPTH_TOPIC[] = "openni_depth";
    static constexpr char COST_TOPIC[] = "cost";
    static constexpr char COLOR_TOPIC[] = "image_color";
    static constexpr char RECT_COLOR_TOPIC[] = "image_rect_color";
    static constexpr char POINTCLOUD_TOPIC[] = "image_points2";
    static constexpr char COLOR_POINTCLOUD_TOPIC[] = "image_points2_color";
    static constexpr char ORGANIZED_POINTCLOUD_TOPIC[] = "organized_image_points2";
    static constexpr char COLOR_ORGANIZED_POINTCLOUD_TOPIC[] = "organized_image_points2_color";
    static constexpr char MONO_CAMERA_INFO_TOPIC[] = "image_mono/camera_info";
    static constexpr char RECT_CAMERA_INFO_TOPIC[] = "image_rect/camera_info";
    static constexpr char COLOR_CAMERA_INFO_TOPIC[] = "image_color/camera_info";
    static constexpr char RECT_COLOR_CAMERA_INFO_TOPIC[] = "image_rect_color/camera_info";
    static constexpr char DEPTH_CAMERA_INFO_TOPIC[] = "depth/camera_info";
    static constexpr char DISPARITY_CAMERA_INFO_TOPIC[] = "disparity/camera_info";
    static constexpr char COST_CAMERA_INFO_TOPIC[] = "cost/camera_info";
    static constexpr char GROUND_SURFACE_IMAGE_TOPIC[] = "image";
    static constexpr char GROUND_SURFACE_INFO_TOPIC[] = "camera_info";
    static constexpr char GROUND_SURFACE_POINT_SPLINE_TOPIC[] = "spline";


    //
    // Device stream control

    void connectStream(crl::multisense::DataSource enableMask);
    void disconnectStream(crl::multisense::DataSource disableMask);
    void stop();

    //
    // Republish camera info messages by publishing the current messages
    // Used whenever the resolution of the camera changes

    void publishAllCameraInfo();

    //
    // CRL sensor API

    crl::multisense::Channel* driver_ = nullptr;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;
    ros::NodeHandle left_nh_;
    ros::NodeHandle right_nh_;
    ros::NodeHandle aux_nh_;
    ros::NodeHandle calibration_nh_;
    ros::NodeHandle ground_surface_nh_;

    //
    // Image transports

    image_transport::ImageTransport  left_mono_transport_;
    image_transport::ImageTransport  right_mono_transport_;
    image_transport::ImageTransport  left_rect_transport_;
    image_transport::ImageTransport  right_rect_transport_;
    image_transport::ImageTransport  left_rgb_transport_;
    image_transport::ImageTransport  left_rgb_rect_transport_;
    image_transport::ImageTransport  depth_transport_;
    image_transport::ImageTransport  ni_depth_transport_;
    image_transport::ImageTransport  disparity_left_transport_;
    image_transport::ImageTransport  disparity_right_transport_;
    image_transport::ImageTransport  disparity_cost_transport_;
    image_transport::ImageTransport  aux_mono_transport_;
    image_transport::ImageTransport  aux_rgb_transport_;
    image_transport::ImageTransport  aux_rect_transport_;
    image_transport::ImageTransport  aux_rgb_rect_transport_;
    image_transport::ImageTransport  ground_surface_transport_;

    //
    // Data publishers

    image_transport::Publisher       left_mono_cam_pub_;
    image_transport::Publisher       right_mono_cam_pub_;
    image_transport::CameraPublisher left_rect_cam_pub_;
    image_transport::CameraPublisher right_rect_cam_pub_;
    image_transport::Publisher       depth_cam_pub_;
    image_transport::Publisher       ni_depth_cam_pub_; // publish depth infomation in the openNI format
    image_transport::Publisher       left_rgb_cam_pub_;
    image_transport::CameraPublisher left_rgb_rect_cam_pub_;
    image_transport::Publisher       aux_rgb_cam_pub_;
    image_transport::Publisher       aux_mono_cam_pub_;
    image_transport::CameraPublisher aux_rect_cam_pub_;
    image_transport::CameraPublisher aux_rgb_rect_cam_pub_;
    image_transport::Publisher       ground_surface_cam_pub_;

    ros::Publisher                   left_mono_cam_info_pub_;
    ros::Publisher                   right_mono_cam_info_pub_;
    ros::Publisher                   left_rect_cam_info_pub_;
    ros::Publisher                   right_rect_cam_info_pub_;
    ros::Publisher                   left_disp_cam_info_pub_;
    ros::Publisher                   right_disp_cam_info_pub_;
    ros::Publisher                   left_cost_cam_info_pub_;
    ros::Publisher                   left_rgb_cam_info_pub_;
    ros::Publisher                   left_rgb_rect_cam_info_pub_;
    ros::Publisher                   depth_cam_info_pub_;
    ros::Publisher                   aux_mono_cam_info_pub_;
    ros::Publisher                   aux_rgb_cam_info_pub_;
    ros::Publisher                   aux_rect_cam_info_pub_;
    ros::Publisher                   aux_rgb_rect_cam_info_pub_;
    ros::Publisher                   ground_surface_info_pub_;

    ros::Publisher                   luma_point_cloud_pub_;
    ros::Publisher                   color_point_cloud_pub_;
    ros::Publisher                   ground_surface_spline_pub_;

    ros::Publisher                   luma_organized_point_cloud_pub_;
    ros::Publisher                   color_organized_point_cloud_pub_;

    image_transport::Publisher       left_disparity_pub_;
    image_transport::Publisher       right_disparity_pub_;
    image_transport::Publisher       left_disparity_cost_pub_;

    ros::Publisher                   left_stereo_disparity_pub_;
    ros::Publisher                   right_stereo_disparity_pub_;

    //
    // Raw data publishers

    ros::Publisher raw_cam_data_pub_;
    ros::Publisher raw_cam_config_pub_;
    ros::Publisher raw_cam_cal_pub_;
    ros::Publisher device_info_pub_;
    ros::Publisher histogram_pub_;

    //
    // Store outgoing messages for efficiency

    sensor_msgs::Image         left_mono_image_;
    sensor_msgs::Image         right_mono_image_;
    sensor_msgs::Image         left_rect_image_;
    sensor_msgs::Image         right_rect_image_;
    sensor_msgs::Image         depth_image_;
    sensor_msgs::Image         ni_depth_image_;
    sensor_msgs::PointCloud2   luma_point_cloud_;
    sensor_msgs::PointCloud2   color_point_cloud_;
    sensor_msgs::PointCloud2   luma_organized_point_cloud_;
    sensor_msgs::PointCloud2   color_organized_point_cloud_;

    sensor_msgs::Image         aux_mono_image_;
    sensor_msgs::Image         left_rgb_image_;
    sensor_msgs::Image         aux_rgb_image_;
    sensor_msgs::Image         left_rgb_rect_image_;
    sensor_msgs::Image         aux_rect_image_;
    sensor_msgs::Image         aux_rgb_rect_image_;

    sensor_msgs::Image         left_disparity_image_;
    sensor_msgs::Image         left_disparity_cost_image_;
    sensor_msgs::Image         right_disparity_image_;

    stereo_msgs::DisparityImage left_stereo_disparity_;
    stereo_msgs::DisparityImage right_stereo_disparity_;

    sensor_msgs::Image         ground_surface_image_;

    multisense_ros::RawCamData raw_cam_data_;

    std::vector<uint8_t> pointcloud_color_buffer_;
    std::vector<uint8_t> pointcloud_rect_color_buffer_;

    //
    // Calibration from sensor

    crl::multisense::system::VersionInfo version_info_;
    crl::multisense::system::DeviceInfo  device_info_;
    std::vector<crl::multisense::system::DeviceMode> device_modes_;

    //
    // Calibration manager

    std::shared_ptr<StereoCalibrationManger> stereo_calibration_manager_;

    //
    // The frame IDs

    const std::string frame_id_origin_;
    const std::string frame_id_head_;
    const std::string frame_id_left_;
    const std::string frame_id_right_;
    const std::string frame_id_aux_;
    const std::string frame_id_rectified_left_;
    const std::string frame_id_rectified_right_;
    const std::string frame_id_rectified_aux_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    //
    // Stream subscriptions

    typedef std::map<crl::multisense::DataSource, int32_t> StreamMapType;
    std::mutex stream_lock_;
    StreamMapType stream_map_;

    //
    // Max distance from the camera for a point to be considered valid

    double pointcloud_max_range_ = 15.0;

    //
    // Histogram tracking

    int64_t last_frame_id_ = -1;

    //
    // The mask used to perform the border clipping of the disparity image

    BorderClip border_clip_type_ = BorderClip::NONE;
    double border_clip_value_ = 0.0;

    //
    // Parameters for drawing ground surface spline

    ground_surface_utilities::SplineDrawParameters spline_draw_params_;

    //
    // Storage of images which we use for pointcloud colorizing

    std::unordered_map<crl::multisense::DataSource, std::shared_ptr<BufferWrapper<crl::multisense::image::Header>>> image_buffers_;

    //
    // Has a 3rd aux color camera

    bool has_aux_camera_ = false;

    //
    // Diagnostics
    diagnostic_updater::Updater diagnostic_updater_;
    void deviceInfoDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void deviceStatusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void ptpStatusDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

    void diagnosticTimerCallback(const ros::TimerEvent&);
    ros::Timer diagnostic_trigger_;

    //
    // Timestamping and timesync settings
    ros::Time imageTimestampToRosTime(uint32_t time_secs, uint32_t time_microsecs);

    bool ptp_time_sync_ = false;
    bool network_time_sync_ = false;
    std::atomic_bool ptp_time_stamp_in_use_;
    int32_t ptp_time_offset_secs_ = 0;
};

}

#endif
