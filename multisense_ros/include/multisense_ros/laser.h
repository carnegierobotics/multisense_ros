/**
 * @file laser.h
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

#ifndef MULTISENSE_ROS_LASER_H
#define MULTISENSE_ROS_LASER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <multisense_lib/MultiSenseChannel.hh>

namespace multisense_ros {

class Laser {
public:
    Laser(crl::multisense::Channel* driver,
          const std::string& tf_prefix);
    ~Laser();

    void scanCallback(const crl::multisense::lidar::Header& header);
    void pointCloudCallback(const crl::multisense::lidar::Header& header);

    static const float EXPECTED_RATE;

private:

    //
    // Device stream control

    void subscribe();
    void unsubscribe();
    void stop();

    //
    // Transform boadcasting
    void publishStaticTransforms(const ros::Time& time);
    void publishSpindleTransform(const float spindle_angle, const float velocity, const ros::Time& time);

    tf::TransformBroadcaster static_tf_broadcaster_;

    void defaultTfPublisher(const ros::TimerEvent& event);

    //
    // Query transforms

    tf::Transform getSpindleTransform(float spindle_angle);

    //
    // Calibration from sensor

    crl::multisense::lidar::Calibration lidar_cal_;

    tf::Transform motor_to_camera_;
    tf::Transform laser_to_spindle_;

    //
    // Frames to Publish
    std::string left_camera_optical_;
    std::string motor_;
    std::string spindle_;
    std::string hokuyo_;

    //
    // Scan publishing

    crl::multisense::Channel *driver_;
    ros::Publisher            scan_pub_;
    std::string               frame_id_;

    //
    // Raw data publishing

    ros::Publisher raw_lidar_data_pub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher raw_lidar_cal_pub_;
    ros::Publisher joint_states_pub_;

    //
    // Keep around for efficiency

    sensor_msgs::LaserScan   laser_msg_;
    sensor_msgs::PointCloud2 point_cloud_;
    sensor_msgs::JointState  joint_states_;

    //
    // Subscriptions

    boost::mutex sub_lock_;
    int32_t      subscribers_;

    //
    // Timer used to publish the default laser transforms

    ros::Timer timer_;

    //
    // Spindle angle used when publishing the default transforms

    float spindle_angle_;


    //
    // Track publishing rates

    ros::Time previous_scan_time_;


}; // class

}// namespace


#endif
