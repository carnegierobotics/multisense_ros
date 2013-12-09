/**
 * @file ros_driver.cpp
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

#include <multisense_ros/laser.h>
#include <multisense_ros/camera.h>
#include <multisense_ros/pps.h>
#include <multisense_ros/imu.h>
#include <multisense_ros/reconfigure.h>
#include <ros/ros.h>

int main(int    argc, 
         char** argvPP)
{
    ros::init(argc, argvPP, "multisense_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    //
    // Get parameters from ROS/command-line

    std::string robot_desc_string;
    std::string sensor_ip;
    int         sensor_mtu;

    if (!nh.getParam("robot_description", robot_desc_string)) {
        ROS_ERROR("Driver: could not find URDF at [robot_description]. Exiting\n");
        return -1;
    }

    nh_private_.param<std::string>("sensor_ip", sensor_ip, "10.66.171.21");
    nh_private_.param<int>("sensor_mtu", sensor_mtu, 7200);

    {
        using namespace crl::multisense;

        Channel *d = Channel::Create(sensor_ip);
    
	if (NULL == d)
            return -2;
        else {

            Status status = d->setMtu(sensor_mtu);
            if (Status_Ok != status)
                ROS_ERROR("Driver: failed to set sensor MTU to %d: %s", 
                          sensor_mtu, Channel::statusString(status));
            else {

                multisense_ros::Laser        laser(d, robot_desc_string);
                multisense_ros::Camera       camera(d);
                multisense_ros::Pps          pps(d);
                multisense_ros::Imu          imu(d);
                multisense_ros::Reconfigure  reconfigure(d, 
                                                         boost::bind(&multisense_ros::Camera::resolutionChanged, &camera));
                ros::spin();
            }

            Channel::Destroy(d);
        }
    }

    return 0;
}
