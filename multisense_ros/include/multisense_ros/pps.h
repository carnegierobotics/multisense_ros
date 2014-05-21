/**
 * @file pps.h
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

#ifndef MULTISENSE_ROS_PPS_H
#define MULTISENSE_ROS_PPS_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <multisense_lib/MultiSenseChannel.hh>

namespace multisense_ros {

class Pps {
public:

    Pps(crl::multisense::Channel* driver);
    ~Pps();

    void ppsCallback(const crl::multisense::pps::Header& header);

private:

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;

    //
    // PPS publisher

    ros::Publisher pps_pub_;

    //
    // Publish control

    int32_t subscribers_;
    void connect();
    void disconnect();
};

}

#endif
