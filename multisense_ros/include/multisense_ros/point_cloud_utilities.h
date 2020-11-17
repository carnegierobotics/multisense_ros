/**
 * @file point_cloud_utilities.h
 *
 * Copyright 2020
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

#ifndef MULTISENSE_ROS_POINT_CLOUD_UTILITY_H
#define MULTISENSE_ROS_POINT_CLOUD_UTILITY_H

#include <arpa/inet.h>

#include <sensor_msgs/PointCloud2.h>

namespace multisense_ros {

template <typename T>
uint8_t message_format();

template <typename T>
sensor_msgs::PointCloud2 initialize_pointcloud(bool dense,
                                               const std::string& frame_id,
                                               const std::string &color_channel)
{
    const auto datatype = message_format<T>();

    sensor_msgs::PointCloud2 point_cloud;
    point_cloud.is_bigendian    = (htonl(1) == 1);
    point_cloud.is_dense        = dense;
    point_cloud.point_step      = 4 * sizeof(T);
    point_cloud.header.frame_id = frame_id;
    point_cloud.fields.resize(4);
    point_cloud.fields[0].name     = "x";
    point_cloud.fields[0].offset   = 0;
    point_cloud.fields[0].count    = 1;
    point_cloud.fields[0].datatype = datatype;
    point_cloud.fields[1].name     = "y";
    point_cloud.fields[1].offset   = sizeof(T);
    point_cloud.fields[1].count    = 1;
    point_cloud.fields[1].datatype = datatype;
    point_cloud.fields[2].name     = "z";
    point_cloud.fields[2].offset   = 2 * sizeof(T);
    point_cloud.fields[2].count    = 1;
    point_cloud.fields[2].datatype = datatype;
    point_cloud.fields[3].name     = color_channel;
    point_cloud.fields[3].offset   = 3 * sizeof(T);
    point_cloud.fields[3].count    = 1;
    point_cloud.fields[3].datatype = datatype;

    return point_cloud;
}


}// namespace

#endif
