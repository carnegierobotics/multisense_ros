/**
 * @file ground_surface_utilities.h
 *
 * Copyright 2021
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

#ifndef MULTISENSE_ROS_GROUND_SURFACE_UTILITIES_H
#define MULTISENSE_ROS_GROUND_SURFACE_UTILITIES_H

#include <vector>
#include <numeric>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <multisense_ros/point_cloud_utilities.h>

namespace ground_surface_utilities {

///
/// @brief Look up table to convert a ground surface class value into into a color for visualization
///        (out-of-bounds -> blue, obstacle -> red, free-space -> green)
/// @param value Raw pixel value resulting from ground surface modeling operation on camera
/// @return RGB value corresponding to the class of the value argument
///
Eigen::Matrix<uint8_t, 3, 1> groundSurfaceClassToPixelColor(const uint8_t value);

///
/// @brief Convert an eigen representation of a pointcloud to a ROS sensor_msgs::PointCloud2 format
/// @param input Pointcloud to convert between eigen and sensor_msg format
/// @param frame_id Base frame ID for resulting sensor_msg
/// @return Pointcloud in sensor_msg format
///
sensor_msgs::PointCloud2 eigenToPointcloud(
    const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &input,
    const std::string &frame_id);

///
/// @brief Struct containing parameters for drawing a pointcloud representation of a B-Spline model
///
struct SplineDrawParameters
{
    /// @brief The max range (along the z dimension / optical axis) to draw the B-spline model
    double max_z_m = 30.0;

    /// @brief The min range (along the z dimension / optical axis) to draw the B-spline model
    double min_z_m = 0.5;

    /// @brief The max width (along the x dimension) to draw the B-spline model
    double max_x_m = 25.0;

    /// @brief The min width (along the x dimension) to draw the B-spline model
    double min_x_m = -25.0;

    /// @brief The resolution to sample the B-Spline model for drawing
    double resolution = 0.1;
};

///
/// @brief Generate a pointcloud representation of a b-spline ground surface model for visualization
/// @param controlGrid Control points grid used to determine interpolated spline values
/// @param splineDrawParams Parameters for drawing a pointcloud representation of a B-Spline model
/// @param pointcloudMaxRange Max range to draw spline model to from camera frame
/// @param xzCellOrigin X,Z cell origin of the spline fitting algorithm in meters
/// @param xzCellSize Size of the X,Z plane containing the spline fit in meters
/// @param minMaxAzimuthAngle Min and max limit to the spline fitting angle in radians, for visualization purposes
/// @param extrinsics Extrinsic transform for stereo pointcloud used during B-Spline modelling
/// @param quadraticParams parameters for the quadratic data transformation prior to spline fitting
/// @param baseline Stereo camera baseline in meters
/// @return Eigen representation of spline pointcloud at regularly sample x/z intervals
///
std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> convertSplineToPointcloud(
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &controlGrid,
    const SplineDrawParameters &splineDrawParams,
    const double pointcloudMaxRange,
    const float* xzCellOrigin,
    const float* xzCellSize,
    const float* minMaxAzimuthAngle,
    const float* extrinsics,
    const float* quadraticParams,
    const float baseline);

} // namespace ground_surface_utilities

#endif
