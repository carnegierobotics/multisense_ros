/**
 * @file camera.h
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

#ifndef MULTISENSE_ROS_CAMERA_H
#define MULTISENSE_ROS_CAMERA_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_ros/RawCamData.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <multisense_lib/MultiSenseChannel.hh>

namespace multisense_ros {

class Camera {
public:
    Camera(crl::multisense::Channel* driver,
           const std::string& tf_prefix);
    ~Camera();

    void resolutionChanged() { queryConfig(); };

    void monoCallback(const crl::multisense::image::Header& header);
    void rectCallback(const crl::multisense::image::Header& header);
    void depthCallback(const crl::multisense::image::Header& header);
    void pointCloudCallback(const crl::multisense::image::Header& header);
    void rawCamDataCallback(const crl::multisense::image::Header& header);
    void colorImageCallback(const crl::multisense::image::Header& header);
    void disparityImageCallback(const crl::multisense::image::Header& header);
    void jpegImageCallback(const crl::multisense::image::Header& header);

private:

    //
    // Device stream control

    void connectStream(crl::multisense::DataSource enableMask);
    void disconnectStream(crl::multisense::DataSource disableMask);
    void stop();

    //
    // Query sensor status and calibration

    void queryConfig();

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;
    ros::NodeHandle left_nh_;
    ros::NodeHandle right_nh_;

    //
    // Image transports

    image_transport::ImageTransport  left_mono_transport_;
    image_transport::ImageTransport  right_mono_transport_;
    image_transport::ImageTransport  left_rect_transport_;
    image_transport::ImageTransport  right_rect_transport_;
    image_transport::ImageTransport  left_rgb_transport_;
    image_transport::ImageTransport  left_rgb_rect_transport_;
    image_transport::ImageTransport  depth_transport_;
    image_transport::ImageTransport  disparity_left_transport_;
    image_transport::ImageTransport  disparity_right_transport_;
    image_transport::ImageTransport  disparity_cost_transport_;

    //
    // Data publishers

    sensor_msgs::CameraInfo          left_rect_cam_info_;
    sensor_msgs::CameraInfo          right_rect_cam_info_;
    sensor_msgs::CameraInfo          left_rgb_rect_cam_info_;

    image_transport::Publisher       left_mono_cam_pub_;
    image_transport::Publisher       right_mono_cam_pub_;
    image_transport::CameraPublisher left_rect_cam_pub_;
    image_transport::CameraPublisher right_rect_cam_pub_;
    image_transport::CameraPublisher depth_cam_pub_;
    image_transport::Publisher       left_rgb_cam_pub_;
    image_transport::CameraPublisher left_rgb_rect_cam_pub_;

    ros::Publisher                   luma_point_cloud_pub_;
    ros::Publisher                   color_point_cloud_pub_;

    image_transport::Publisher       left_disparity_pub_;
    image_transport::Publisher       right_disparity_pub_;
    image_transport::Publisher       left_disparity_cost_pub_;

    //
    // Raw data publishers

    ros::Publisher raw_cam_data_pub_;
    ros::Publisher raw_cam_config_pub_;
    ros::Publisher raw_cam_cal_pub_;
    ros::Publisher device_info_pub_;

    //
    // Store outgoing messages for efficiency

    sensor_msgs::Image         left_mono_image_;
    sensor_msgs::Image         right_mono_image_;
    sensor_msgs::Image         left_rect_image_;
    sensor_msgs::Image         right_rect_image_;
    sensor_msgs::Image         depth_image_;
    sensor_msgs::PointCloud2   luma_point_cloud_;
    sensor_msgs::PointCloud2   color_point_cloud_;

    sensor_msgs::Image         left_luma_image_;
    sensor_msgs::Image         left_rgb_image_;
    sensor_msgs::Image         left_rgb_rect_image_;

    sensor_msgs::Image         left_disparity_image_;
    sensor_msgs::Image         left_disparity_cost_image_;
    sensor_msgs::Image         right_disparity_image_;

    bool                       got_raw_cam_left_;
    bool                       got_left_luma_;
    int64_t                    left_luma_frame_id_;
    int64_t                    left_rect_frame_id_;
    int64_t                    left_rgb_rect_frame_id_;
    multisense_ros::RawCamData raw_cam_data_;

    //
    // Calibration from sensor

    crl::multisense::system::VersionInfo version_info_;
    crl::multisense::system::DeviceInfo  device_info_;
    crl::multisense::image::Config       image_config_;
    crl::multisense::image::Calibration  image_calibration_;

    //
    // For local rectification of color images
    
    boost::mutex cal_lock_;
    CvMat *calibration_map_left_1_;
    CvMat *calibration_map_left_2_;

    //
    // The frame IDs
    
    std::string frame_id_left_;
    std::string frame_id_right_;
    
    //
    // For pointcloud generation

    std::vector<float>            disparity_buff_;
    std::vector<cv::Vec3f>        points_buff_;
    cv::Mat_<double>              q_matrix_;
    uint32_t                      pc_border_clip_;
    float                         pc_max_range_;
    bool                          pc_color_frame_sync_;

    //
    // Stream subscriptions
    
    typedef std::map<crl::multisense::DataSource, int32_t> StreamMapType;
    boost::mutex stream_lock_;
    StreamMapType stream_map_;
};

}

#endif
