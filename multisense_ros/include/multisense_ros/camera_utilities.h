/**
 * @file camera_utilities.h
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

#ifndef MULTISENSE_ROS_CAMERA_UTILITIES_H
#define MULTISENSE_ROS_CAMERA_UTILITIES_H

#include <tuple>
#include <mutex>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <multisense_lib/MultiSenseChannel.hh>
#include <multisense_lib/MultiSenseTypes.hh>

namespace multisense_ros {

enum class BorderClip {NONE, RECTANGULAR, CIRCULAR};

struct RectificationRemapT
{
    cv::Mat map1;
    cv::Mat map2;
};

template <typename T>
class BufferWrapper
{
public:
    BufferWrapper(crl::multisense::Channel* driver,
                 const T &data):
        driver_(driver),
        callback_buffer_(driver->reserveCallbackBuffer()),
        data_(data)
    {
    }

    ~BufferWrapper()
    {
        driver_->releaseCallbackBuffer(callback_buffer_);
    }

    const T &data() const noexcept
    {
        return data_;
    }

private:

    BufferWrapper(const BufferWrapper&) = delete;
    BufferWrapper operator=(const BufferWrapper&) = delete;

    crl::multisense::Channel * driver_;
    void* callback_buffer_;
    const T data_;

};

void ycbcrToBgr(const crl::multisense::image::Header &luma,
                const crl::multisense::image::Header &chroma,
                uint8_t *output);

cv::Vec3b ycbcrToBgr(const crl::multisense::image::Header &luma,
                     const crl::multisense::image::Header &chroma,
                     size_t u,
                     size_t v);

Eigen::Matrix4d makeQ(const crl::multisense::image::Config& config,
                      const crl::multisense::image::Calibration& calibration,
                      const crl::multisense::system::DeviceInfo& device_info);

sensor_msgs::CameraInfo makeCameraInfo(const crl::multisense::image::Config& config,
                                       const crl::multisense::image::Calibration::Data& calibration,
                                       const crl::multisense::system::DeviceInfo& device_info,
                                       bool scale_calibration);

RectificationRemapT makeRectificationRemap(const crl::multisense::image::Config& config,
                                           const crl::multisense::image::Calibration::Data& calibration,
                                           const crl::multisense::system::DeviceInfo& device_info);
class StereoCalibrationManger
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoCalibrationManger(const crl::multisense::image::Config& config,
                            const crl::multisense::image::Calibration& calibration,
                            const crl::multisense::system::DeviceInfo& device_info);

    void updateConfig(const crl::multisense::image::Config& config);

    crl::multisense::image::Config config() const;

    ///
    /// @brief Q stereo reprojection matrix see:
    /// https://docs.opencv.org/4.3.0/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
    ///
    Eigen::Matrix4d Q() const;

    ///
    /// @brief Translation which transforms points from the right camera frame into the left camera frame
    ///
    double T() const;

    ///
    /// @brief Translation which transforms points from the aux camera frame into the left camera frame
    ///
    double aux_T() const;

    sensor_msgs::CameraInfo leftCameraInfo(const std::string& frame_id, const ros::Time& stamp) const;
    sensor_msgs::CameraInfo rightCameraInfo(const std::string& frame_id, const ros::Time& stamp) const;
    sensor_msgs::CameraInfo auxCameraInfo(const std::string& frame_id, const ros::Time& stamp) const;

    std::shared_ptr<RectificationRemapT> leftRemap() const;
    std::shared_ptr<RectificationRemapT> rightRemap() const;

private:

    crl::multisense::image::Config config_;
    const crl::multisense::image::Calibration calibration_;
    const crl::multisense::system::DeviceInfo& device_info_;

    //
    // Protect our cache during calibration updates

    mutable std::mutex mutex_;

    //
    // Cache for quick queries

    Eigen::Matrix4d q_matrix_;

    sensor_msgs::CameraInfo left_camera_info_;
    sensor_msgs::CameraInfo right_camera_info_;
    sensor_msgs::CameraInfo aux_camera_info_;

    std::shared_ptr<RectificationRemapT> left_remap_;
    std::shared_ptr<RectificationRemapT> right_remap_;
};

}// namespace

#endif
