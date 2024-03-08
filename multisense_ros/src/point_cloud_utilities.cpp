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

#include <multisense_ros/point_cloud_utilities.h>

namespace multisense_ros {

template <>
uint8_t messageFormat<int8_t>()
{
    return sensor_msgs::PointField::INT8;
}

template <>
uint8_t messageFormat<uint8_t>()
{
    return sensor_msgs::PointField::UINT8;
}

template <>
uint8_t messageFormat<int16_t>()
{
    return sensor_msgs::PointField::INT16;
}

template <>
uint8_t messageFormat<uint16_t>()
{
    return sensor_msgs::PointField::UINT16;
}

template <>
uint8_t messageFormat<int32_t>()
{
    return sensor_msgs::PointField::INT32;
}

template <>
uint8_t messageFormat<uint32_t>()
{
    return sensor_msgs::PointField::UINT32;
}

template <>
uint8_t messageFormat<float>()
{
    return sensor_msgs::PointField::FLOAT32;
}

template <>
uint8_t messageFormat<double>()
{
    return sensor_msgs::PointField::FLOAT64;
}

}// namespace
