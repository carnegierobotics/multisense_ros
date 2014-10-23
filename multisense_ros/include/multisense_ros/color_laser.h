/**
 * @file color_laser.h
 *
 * Copyright 2014
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

#ifndef MULTISENSE_ROS_COLOR_LASER_H
#define MULTISENSE_ROS_COLOR_LASER_H

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace multisense_ros{

class ColorLaser
{
    public:

        ColorLaser(
            ros::NodeHandle& nh
        );

        ~ColorLaser();


        //
        // Callbacks for subscriptions to ROS topics

        void colorImageCallback(const sensor_msgs::Image::ConstPtr& message);
        void laserPointCloudCallback(sensor_msgs::PointCloud2::Ptr message);
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message);

    private:

        //
        // Callback used to setup subscribers once there is a subscription
        // to the lidar_points2_color topic

        void startStreaming();
        void stopStreaming();

        //
        // Messages for local storage of sensor data

        sensor_msgs::Image       color_image_;
        sensor_msgs::CameraInfo  camera_info_;

        sensor_msgs::PointCloud2 color_laser_pointcloud_;

        //
        // Publisher for the colorized laser point cloud

        ros::Publisher color_laser_publisher_;

        //
        // Subscribers for image, point cloud, and camera info topics

        ros::Subscriber color_image_sub_;
        ros::Subscriber laser_pointcloud_sub_;
        ros::Subscriber camera_info_sub_;

        //
        // Node handle used for publishing/subscribing to topics

        ros::NodeHandle node_handle_;

        //
        // Mutex to assure callbacks don't interfere with one another

        boost::mutex data_lock_;

        //
        // The number of channels in our color image

        uint8_t image_channels_;
};

}// namespace

#endif

