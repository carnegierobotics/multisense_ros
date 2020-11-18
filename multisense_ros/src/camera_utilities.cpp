/**
 * @file camera_utilities.cpp
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

#include <algorithm>

#include <sensor_msgs/distortion_models.h>

#include <multisense_ros/camera_utilities.h>

namespace multisense_ros {

namespace {

struct ScaleT
{
    double x_scale = 0.0;
    double y_scale = 0.0;
    double cx_offset = 0.0;
    double cy_offset = 0.0;
};

ScaleT compute_scale(const crl::multisense::image::Config &config,
                     const crl::multisense::system::DeviceInfo& device_info)
{

    const auto crop = config.camMode() == 2000;

    const double x_scale = 1.0 / ((static_cast<double>(device_info.imagerWidth) /
                                   static_cast<double>(config.width())));

    const double y_scale = 1.0 / ((static_cast<double>(device_info.imagerHeight) /
                                   static_cast<double>(config.height())));

    //
    // In crop mode we dont want to scale our fx/fy and cx/cy value. We just want to offset our cx/cy values
    // by the current crop offset. This is because the pixel size does not change in crop mode, and instead a cropped
    // region of the original image is returned

    return ScaleT{crop ? 1.0 : x_scale,
                  crop ? 1.0 : y_scale,
                  crop ? config.offset() : 0.0,
                  crop ? config.offset() : 0.0};
}

}// namespace

void ycbcrToBgr(const crl::multisense::image::Header &luma,
                const crl::multisense::image::Header &chroma,
                uint8_t *output)
{
    const size_t rgb_stride = luma.width * 3;

    for(uint32_t y=0; y< luma.height; ++y)
    {
        for(uint32_t x=0; x< luma.width; ++x)
        {
            const auto &pixel = ycbcrToBgr(luma, chroma, x, y);

            const size_t rgb_offset = (y * rgb_stride) + (3 * x);

            output[rgb_offset + 0] = pixel[0];
            output[rgb_offset + 1] = pixel(1);
            output[rgb_offset + 2] = pixel(2);
        }
    }
}


cv::Vec3b ycbcrToBgr(const crl::multisense::image::Header &luma,
                     const crl::multisense::image::Header &chroma,
                     size_t u,
                     size_t v)
{
    const uint8_t *lumaP = reinterpret_cast<const uint8_t*>(luma.imageDataP);
    const uint8_t *chromaP = reinterpret_cast<const uint8_t*>(chroma.imageDataP);

    const size_t luma_offset = (v * luma.width) + u;
    const size_t chroma_offset = 2 * (((v/2) * (luma.width/2)) + (u/2));

    const float px_y = static_cast<float>(lumaP[luma_offset]);
    const float px_cb = static_cast<float>(chromaP[chroma_offset+0]) - 128.0f;
    const float px_cr = static_cast<float>(chromaP[chroma_offset+1]) - 128.0f;

    float px_r  = px_y + 1.402f   * px_cr;
    float px_g  = px_y - 0.34414f * px_cb - 0.71414f * px_cr;
    float px_b  = px_y + 1.772f   * px_cb;

    if (px_r < 0.0f)        px_r = 0.0f;
    else if (px_r > 255.0f) px_r = 255.0f;
    if (px_g < 0.0f)        px_g = 0.0f;
    else if (px_g > 255.0f) px_g = 255.0f;
    if (px_b < 0.0f)        px_b = 0.0f;
    else if (px_b > 255.0f) px_b = 255.0f;

    return cv::Vec3b{static_cast<uint8_t>(px_b), static_cast<uint8_t>(px_g), static_cast<uint8_t>(px_r)};
}

Eigen::Matrix4d makeQ(const crl::multisense::image::Config& config,
                      const crl::multisense::image::Calibration& calibration,
                      const crl::multisense::system::DeviceInfo& device_info)
{
    Eigen::Matrix4d q_matrix = Eigen::Matrix4d::Zero();

    const auto scale = compute_scale(config, device_info);

    //
    // Compute the Q matrix here, as image_geometery::StereoCameraModel does
    // not allow for non-square pixels.
    //
    //  FyTx    0     0    -FyCxTx
    //   0     FxTx   0    -FxCyTx
    //   0      0     0     FxFyTx
    //   0      0    -Fy    Fy(Cx - Cx')

    q_matrix(0,0) =  config.fy() * config.tx();
    q_matrix(1,1) =  config.fx() * config.tx();
    q_matrix(0,3) = -config.fy() * config.cx() * config.tx();
    q_matrix(1,3) = -config.fx() * config.cy() * config.tx();
    q_matrix(2,3) =  config.fx() * config.fy() * config.tx();
    q_matrix(3,2) = -config.fy();
    q_matrix(3,3) =  config.fy() * (calibration.left.P[0][2] * scale.x_scale - calibration.right.P[0][2] * scale.x_scale);

    return q_matrix;
}

sensor_msgs::CameraInfo makeCameraInfo(const crl::multisense::image::Config& config,
                                       const crl::multisense::image::Calibration::Data& calibration,
                                       const crl::multisense::system::DeviceInfo& device_info,
                                       bool scale_calibration)
{
    const auto scale = scale_calibration ? compute_scale(config, device_info) : ScaleT{1.0, 1.0, 0.0, 0.0};

    sensor_msgs::CameraInfo camera_info;

    camera_info.P[0] = calibration.P[0][0] * scale.x_scale;
    camera_info.P[1] = calibration.P[0][1];
    camera_info.P[2] = calibration.P[0][2] * scale.x_scale + scale.cx_offset;
    camera_info.P[3] = calibration.P[0][3] * scale.x_scale;
    camera_info.P[4] = calibration.P[1][0];
    camera_info.P[5] = calibration.P[1][1] * scale.y_scale;
    camera_info.P[6] = calibration.P[1][2] * scale.y_scale + scale.cy_offset;
    camera_info.P[7] = calibration.P[1][3];
    camera_info.P[8] = calibration.P[2][0];
    camera_info.P[9] = calibration.P[2][1];
    camera_info.P[10] = calibration.P[2][2];
    camera_info.P[11] = calibration.P[2][3];

    camera_info.K[0] = calibration.M[0][0] * scale.x_scale;
    camera_info.K[1] = calibration.M[0][1];
    camera_info.K[2] = calibration.M[0][2] * scale.x_scale + scale.cx_offset;
    camera_info.K[3] = calibration.M[1][0];
    camera_info.K[4] = calibration.M[1][1] * scale.y_scale;
    camera_info.K[5] = calibration.M[1][2] * scale.y_scale + scale.cy_offset;
    camera_info.K[6] = calibration.M[2][0];
    camera_info.K[7] = calibration.M[2][1];
    camera_info.K[8] = calibration.M[2][2];

    camera_info.R[0] = calibration.R[0][0];
    camera_info.R[1] = calibration.R[0][1];
    camera_info.R[2] = calibration.R[0][2];
    camera_info.R[3] = calibration.R[1][0];
    camera_info.R[4] = calibration.R[1][1];
    camera_info.R[5] = calibration.R[1][2];
    camera_info.R[6] = calibration.R[2][0];
    camera_info.R[7] = calibration.R[2][1];
    camera_info.R[8] = calibration.R[2][2];

    //
    // Distortion coefficients follow OpenCV's convention.
    // k1, k2, p1, p2, k3, k4, k5, k6

    camera_info.D.resize(8);
    for (size_t i=0 ; i < 8 ; ++i)
    {
        camera_info.D[i] = calibration.D[i];
    }

    //
    // MultiSense cameras support both the full 8 parameter rational_polynomial
    // model and the simplified 5 parameter plum_bob model. If the last 3
    // parameters of the distortion model are 0 then the camera is using
    // the simplified plumb_bob model

    if (calibration.D[7] == 0.0 && calibration.D[6] == 0.0 && calibration.D[5] == 0.0)
    {
        camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    }
    else
    {
        camera_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
    }

    return camera_info;
}

RectificationRemapT makeRectificationRemap(const crl::multisense::image::Config& config,
                                           const crl::multisense::image::Calibration::Data& calibration,
                                           const crl::multisense::system::DeviceInfo& device_info)
{
    RectificationRemapT remap;

    const auto scale = compute_scale(config, device_info);

    const cv::Matx33d K(calibration.M[0][0] * scale.x_scale,
                        calibration.M[0][1],
                        calibration.M[0][2] * scale.x_scale + scale.cx_offset,
                        calibration.M[1][0],
                        calibration.M[1][1] * scale.y_scale,
                        calibration.M[1][2] * scale.y_scale + scale.cy_offset,
                        calibration.M[2][0],
                        calibration.M[2][1],
                        calibration.M[2][2]);

    const cv::Matx33d R(calibration.R[0][0],
                        calibration.R[0][1],
                        calibration.R[0][2],
                        calibration.R[1][0],
                        calibration.R[1][1],
                        calibration.R[1][2],
                        calibration.R[2][0],
                        calibration.R[2][1],
                        calibration.R[2][2]);

    const cv::Matx34d P(calibration.P[0][0] * scale.x_scale,
                        calibration.P[0][1],
                        calibration.P[0][2] * scale.x_scale + scale.cx_offset,
                        calibration.P[0][3] * scale.x_scale,
                        calibration.P[1][0],
                        calibration.P[1][1] * scale.y_scale,
                        calibration.P[1][2] * scale.y_scale + scale.cy_offset,
                        calibration.P[1][3],
                        calibration.P[2][0],
                        calibration.P[2][1],
                        calibration.P[2][2],
                        calibration.P[2][3]);

    const bool plumbob = calibration.D[7] == 0.0 && calibration.D[6] == 0.0 && calibration.D[5] == 0.0;

    cv::Mat D(plumbob ? 5 : 8, 1, CV_64FC1);
    for (int i = 0; i < D.rows ; ++i)
    {
        D.at<double>(i) = calibration.D[i];
    }

    cv::initUndistortRectifyMap(K, D, R, P, cv::Size(config.width(), config.height()), CV_32FC1, remap.map1, remap.map2);

    return remap;
}

StereoCalibrationManger::StereoCalibrationManger(const crl::multisense::image::Config& config,
                                                 const crl::multisense::image::Calibration& calibration,
                                                 const crl::multisense::system::DeviceInfo& device_info):
    config_(config),
    calibration_(calibration),
    device_info_(device_info),
    q_matrix_(makeQ(config_, calibration_, device_info_)),
    left_camera_info_(makeCameraInfo(config_, calibration_.left, device_info_, true)),
    right_camera_info_(makeCameraInfo(config_, calibration_.right, device_info_, true)),
    aux_camera_info_(makeCameraInfo(config_, calibration_.aux, device_info_, false)),
    left_remap_(std::make_shared<RectificationRemapT>(makeRectificationRemap(config_, calibration_.left, device_info_))),
    right_remap_(std::make_shared<RectificationRemapT>(makeRectificationRemap(config_, calibration_.right, device_info_)))
{
}

void StereoCalibrationManger::updateConfig(const crl::multisense::image::Config& config)
{
    //
    // Only update the calibration if the resolution changed.

    if (config_.width() == config.width() && config_.height() == config.height())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        return;
    }

    auto q_matrix = makeQ(config, calibration_, device_info_);
    auto left_camera_info = makeCameraInfo(config, calibration_.left, device_info_, true);
    auto right_camera_info = makeCameraInfo(config, calibration_.right, device_info_, true);
    auto aux_camera_info_ = makeCameraInfo(config, calibration_.right, device_info_, false);
    auto left_remap = std::make_shared<RectificationRemapT>(makeRectificationRemap(config, calibration_.left, device_info_));
    auto right_remap = std::make_shared<RectificationRemapT>(makeRectificationRemap(config, calibration_.right, device_info_));

    std::lock_guard<std::mutex> lock(mutex_);

    config_ = config;
    q_matrix_ = std::move(q_matrix);
    left_camera_info_ = std::move(left_camera_info);
    right_camera_info_ = std::move(right_camera_info);
    left_remap_ = left_remap;
    right_remap_ = right_remap;
}

crl::multisense::image::Config StereoCalibrationManger::config() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return config_;
}

Eigen::Matrix4d StereoCalibrationManger::Q() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return q_matrix_;
}

double StereoCalibrationManger::T() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    //
    // The right camera projection matrix is of the form:
    //
    // [fx,  0, cx, t * fx]
    // [ 0, fy, cy, 0     ]
    // [ 0,  0,  1, 0     ]
    //
    // divide the t * fx term by fx to get t

    return right_camera_info_.P[3] / right_camera_info_.P[0];
}

double StereoCalibrationManger::aux_T() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    //
    // The aux camera projection matrix is of the form:
    //
    // [fx,  0, cx, t * fx]
    // [ 0, fy, cy, 0     ]
    // [ 0,  0,  1, 0     ]
    //
    // divide the t * fx term by fx to get t

    return aux_camera_info_.P[3] / aux_camera_info_.P[0];
}

sensor_msgs::CameraInfo StereoCalibrationManger::leftCameraInfo(const std::string& frame_id,
                                                                const ros::Time& stamp) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto camera_info = left_camera_info_;
    camera_info.header.frame_id = frame_id;
    camera_info.header.stamp = stamp;

    return camera_info;
}

sensor_msgs::CameraInfo StereoCalibrationManger::rightCameraInfo(const std::string& frame_id,
                                                                 const ros::Time& stamp) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto camera_info = right_camera_info_;
    camera_info.header.frame_id = frame_id;
    camera_info.header.stamp = stamp;

    return camera_info;
}

sensor_msgs::CameraInfo StereoCalibrationManger::auxCameraInfo(const std::string& frame_id,
                                                               const ros::Time& stamp) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto camera_info = aux_camera_info_;
    camera_info.header.frame_id = frame_id;
    camera_info.header.stamp = stamp;

    return camera_info;
}

std::shared_ptr<RectificationRemapT> StereoCalibrationManger::leftRemap() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return left_remap_;
}

std::shared_ptr<RectificationRemapT> StereoCalibrationManger::rightRemap() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return right_remap_;
}

}// namespace
