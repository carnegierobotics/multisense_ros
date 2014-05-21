/**
 * @file laser.h
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
          const std::string& tf_prefix,
          const std::string& robot_desc);
    ~Laser();

    void scanCallback(const crl::multisense::lidar::Header& header);
    void pointCloudCallback(const crl::multisense::lidar::Header& header);
    void pointCloudCallbackT(const crl::multisense::lidar::Header& header);

    static const float EXPECTED_RATE;
    
private:

    //
    // Device stream control

    void subscribe();
    void unsubscribe();
    void stop();

    //
    // Transform boadcasting 
    void publishStaticTransforms(ros::Time time);
    tf::Transform publishSpindleTransform(float spindle_angle, ros::Time time, bool publish=true);

    tf::TransformBroadcaster static_tf_broadcaster_;
    tf::TransformBroadcaster spindle_tf_broadcaster_;

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

    //
    // Keep around for efficiency

    sensor_msgs::LaserScan   laser_msg_;
    sensor_msgs::PointCloud2 point_cloud_;

    //
    // Subscriptions
    
    boost::mutex sub_lock_;
    int32_t      subscribers_;

}; // class

}// namespace


#endif
