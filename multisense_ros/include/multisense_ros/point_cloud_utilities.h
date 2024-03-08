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
#include <Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>

namespace multisense_ros {

template <typename T>
uint8_t messageFormat();

template <typename PointT, typename ColorT>
sensor_msgs::PointCloud2 initializePointcloud(bool dense,
                                               const std::string& frame_id,
                                               const std::string &color_channel)
{
    const auto pointDatatype = messageFormat<PointT>();
    const auto colorDatatype = messageFormat<ColorT>();

    sensor_msgs::PointCloud2 point_cloud;
    point_cloud.is_bigendian    = (htonl(1) == 1);
    point_cloud.is_dense        = dense;
    point_cloud.point_step      = 3 * sizeof(PointT) + sizeof(ColorT);
    point_cloud.header.frame_id = frame_id;
    point_cloud.fields.resize(4);
    point_cloud.fields[0].name     = "x";
    point_cloud.fields[0].offset   = 0;
    point_cloud.fields[0].count    = 1;
    point_cloud.fields[0].datatype = pointDatatype;
    point_cloud.fields[1].name     = "y";
    point_cloud.fields[1].offset   = sizeof(PointT);
    point_cloud.fields[1].count    = 1;
    point_cloud.fields[1].datatype = pointDatatype;
    point_cloud.fields[2].name     = "z";
    point_cloud.fields[2].offset   = 2 * sizeof(PointT);
    point_cloud.fields[2].count    = 1;
    point_cloud.fields[2].datatype = pointDatatype;
    point_cloud.fields[3].name     = color_channel;
    point_cloud.fields[3].offset   = 3 * sizeof(PointT);
    point_cloud.fields[3].count    = 1;
    point_cloud.fields[3].datatype = colorDatatype;

    return point_cloud;
}

template <typename ColorT>
void writePoint(sensor_msgs::PointCloud2 &pointcloud, const size_t index, const Eigen::Vector3f &point, const ColorT color)
{
    assert(index < pointcloud.data.size());

    float* cloudP = reinterpret_cast<float*>(&(pointcloud.data[index * pointcloud.point_step]));

    assert(pointcloud.fields[0].datatype == messageFormat<float>());
    assert(pointcloud.fields[1].datatype == messageFormat<float>());
    assert(pointcloud.fields[2].datatype == messageFormat<float>());
    cloudP[0] = point[0];
    cloudP[1] = point[1];
    cloudP[2] = point[2];

    assert(pointcloud.fields[3].datatype == messageFormat<ColorT>());

    ColorT* colorP = reinterpret_cast<ColorT*>(&(cloudP[3]));
    colorP[0] = color;
}

void writePoint(sensor_msgs::PointCloud2 &pointcloud,
                size_t pointcloud_index,
                const Eigen::Vector3f &point,
                size_t image_index,
                const uint32_t bitsPerPixel,
                const void* imageDataP);

}// namespace

#endif
