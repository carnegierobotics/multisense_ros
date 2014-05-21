/**
 * @file imu.h
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

#ifndef MULTISENSE_ROS_IMU_H
#define MULTISENSE_ROS_IMU_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <multisense_lib/MultiSenseChannel.hh>

namespace multisense_ros {

class Imu {
public:

    Imu(crl::multisense::Channel* driver);
    ~Imu();

    void imuCallback(const crl::multisense::imu::Header& header);

private:

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;
    ros::NodeHandle imu_nh_;

    //
    // IMU publishers

    ros::Publisher accelerometer_pub_;
    ros::Publisher gyroscope_pub_;
    ros::Publisher magnetometer_pub_;

    //
    // Publish control

    boost::mutex sub_lock_;
    int32_t accel_subscribers_;
    int32_t gyro_subscribers_;
    int32_t mag_subscribers_;
    int32_t total_subscribers_;
    void connect(crl::multisense::imu::Sample::Type type);
    void disconnect(crl::multisense::imu::Sample::Type type);
};

}

#endif
