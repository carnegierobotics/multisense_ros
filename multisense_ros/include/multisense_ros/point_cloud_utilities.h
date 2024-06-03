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

#include <type_traits>

#include <arpa/inet.h>
#include <Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>

namespace multisense_ros {

template <typename T>
uint8_t messageFormat();

template <typename ColorT>
typename std::enable_if<!std::is_same<ColorT, void>::value, void>::type
 initPointcloudColorChannel(sensor_msgs::PointCloud2& point_cloud,
                            const size_t offset,
                            const std::string& color_channel)
{
    assert(point_cloud.fields.size() >= 4);

    const auto color_datatype = messageFormat<ColorT>();

    point_cloud.fields[3].name     = color_channel;
    point_cloud.fields[3].offset   = offset;
    point_cloud.fields[3].count    = 1;
    point_cloud.fields[3].datatype = color_datatype;

    point_cloud.point_step += sizeof(ColorT);
}

template <typename ColorT>
typename std::enable_if<std::is_same<ColorT, void>::value, void>::type
initPointcloudColorChannel(sensor_msgs::PointCloud2& point_cloud,
                           const size_t offset,
                           const std::string& color_channel)
{
    (void) point_cloud;
    (void) offset;
    (void) color_channel;
    return;
}

template <typename PointT, typename ColorT>
void initializePointcloud (sensor_msgs::PointCloud2& point_cloud,
                           const ros::Time& stamp,
                           const size_t width,
                           const size_t height,
                           const bool dense,
                           const std::string& frame_id,
                           const std::string& color_channel)
{
    const auto point_datatype = messageFormat<PointT>();

    const bool has_color = not std::is_same<ColorT, void>::value;

    point_cloud.is_bigendian    = (htonl(1) == 1);
    point_cloud.is_dense        = dense;
    point_cloud.point_step = 0;
    point_cloud.header.frame_id = frame_id;
    point_cloud.header.stamp = stamp;
    point_cloud.width = width;
    point_cloud.height = height;

    if (has_color)
    {
        point_cloud.fields.resize(4);
    }
    else
    {
        point_cloud.fields.resize(3);
    }

    point_cloud.fields[0].name     = "x";
    point_cloud.fields[0].offset   = point_cloud.point_step;
    point_cloud.fields[0].count    = 1;
    point_cloud.fields[0].datatype = point_datatype;
    point_cloud.point_step += sizeof(PointT);

    point_cloud.fields[1].name     = "y";
    point_cloud.fields[1].offset   = point_cloud.point_step;
    point_cloud.fields[1].count    = 1;
    point_cloud.fields[1].datatype = point_datatype;
    point_cloud.point_step += sizeof(PointT);

    point_cloud.fields[2].name     = "z";
    point_cloud.fields[2].offset   = point_cloud.point_step;
    point_cloud.fields[2].count    = 1;
    point_cloud.fields[2].datatype = point_datatype;
    point_cloud.point_step += sizeof(PointT);

    if (has_color)
    {
        initPointcloudColorChannel<ColorT>(point_cloud,
                                           point_cloud.point_step,
                                           color_channel);
    }

    point_cloud.row_step = width * point_cloud.point_step;
    point_cloud.data.resize(width * height * point_cloud.point_step);
}

template <typename PointT, typename ColorT>
sensor_msgs::PointCloud2 initializePointcloud(const ros::Time& stamp,
                                              const size_t width,
                                              const size_t height,
                                              const bool dense,
                                              const std::string& frame_id,
                                              const std::string& color_channel)
{
    sensor_msgs::PointCloud2 point_cloud;

    initializePointcloud<PointT, ColorT>(point_cloud,
                                         stamp,
                                         width,
                                         height,
                                         dense,
                                         frame_id,
                                         color_channel);

    return point_cloud;
}

void writePoint(sensor_msgs::PointCloud2 &pointcloud, const size_t index, const Eigen::Vector3f &point);

template <typename ColorT>
void writePoint(sensor_msgs::PointCloud2 &pointcloud, const size_t index, const Eigen::Vector3f &point, const ColorT &color)
{
    writePoint(pointcloud, index, point);

    assert(pointcloud.fields.size() == 4);
    assert(pointcloud.fields[3].datatype == messageFormat<ColorT>());

    ColorT* colorP = reinterpret_cast<ColorT*>(&(pointcloud.data[index * pointcloud.point_step + pointcloud.fields[3].offset]));
    colorP[0] = color;
}

void writePoint(sensor_msgs::PointCloud2 &pointcloud,
                const size_t pointcloud_index,
                const Eigen::Vector3f &point,
                const size_t image_index,
                const uint32_t bitsPerPixel,
                const void* imageDataP);

}// namespace

#endif
