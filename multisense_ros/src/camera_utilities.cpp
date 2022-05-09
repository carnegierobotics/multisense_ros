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
    double x_scale = 1.0;
    double y_scale = 1.0;
    double cx_offset = 0.0;
    double cy_offset = 0.0;
};

ScaleT compute_scale(const crl::multisense::image::Config &config,
                     const crl::multisense::system::DeviceInfo& device_info)
{
    const auto crop = config.camMode() == 2000 &&
                      (device_info.imagerType == crl::multisense::system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY ||
                      device_info.imagerType == crl::multisense::system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR);

    // crop mode causes the imager to behave completely like a CMV2000, but the device info imager height does not get modified
    const auto imagerHeight = crop ? 1088 : device_info.imagerHeight;

    const double x_scale = 1.0 / ((static_cast<double>(device_info.imagerWidth) /
                                   static_cast<double>(config.width())));

    const double y_scale = 1.0 / ((static_cast<double>(imagerHeight) /
                                   static_cast<double>(config.height())));

    //
    // In crop mode we want to offset our cx/cy values
    // by the current crop offset. This is because the pixel size does not change in crop mode, and instead a cropped
    // region of the original image is returned

    return ScaleT{x_scale,
                  y_scale,
                  0.0,
                  crop ? -config.offset()*y_scale : 0.0};
}

}// namespace

void ycbcrToBgr(const crl::multisense::image::Header &luma,
                const crl::multisense::image::Header &chroma,
                uint8_t *output)
{
    const size_t rgb_stride = luma.width * 3;

    for(uint32_t y=0; y< luma.height; ++y)
    {
        const size_t row_offset = y * rgb_stride;

        for(uint32_t x=0; x< luma.width; ++x)
        {
            memcpy(output + row_offset + (3 * x), ycbcrToBgr<uint8_t>(luma, chroma, x, y).data(), 3);
        }
    }
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
    //
    const auto fx = calibration.left.P[0][0] * scale.x_scale;
    const auto cx = calibration.left.P[0][2] * scale.x_scale + scale.cx_offset;
    const auto fy = calibration.left.P[1][1] * scale.y_scale;
    const auto cy = calibration.left.P[1][2] * scale.y_scale + scale.cy_offset;
    const auto tx = calibration.right.P[0][3] / calibration.right.P[0][0];
    const auto cx_prime = calibration.left.P[0][2] * scale.x_scale + scale.cx_offset;

    q_matrix(0,0) =  fy * tx;
    q_matrix(1,1) =  fx * tx;
    q_matrix(0,3) = -fy * cx * tx;
    q_matrix(1,3) = -fx * cy * tx;
    q_matrix(2,3) =  fx * fy * tx;
    q_matrix(3,2) = -fy;
    q_matrix(3,3) =  fy * (cx - cx_prime);

    return q_matrix;
}

sensor_msgs::CameraInfo makeCameraInfo(const crl::multisense::image::Config& config,
                                       const crl::multisense::image::Calibration::Data& calibration,
                                       const ScaleT &scale)
{
    sensor_msgs::CameraInfo camera_info;

    camera_info.width = config.width();
    camera_info.height = config.height();

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
    left_camera_info_(makeCameraInfo(config_, calibration_.left, compute_scale(config_, device_info_))),
    right_camera_info_(makeCameraInfo(config_, calibration_.right, compute_scale(config_, device_info_))),
    aux_camera_info_(makeCameraInfo(config_, calibration_.aux, config_.cameraProfile() == crl::multisense::Full_Res_Aux_Cam ?
                                                               ScaleT{1., 1., 0., 0.} : compute_scale(config_, device_info_))),
    left_remap_(std::make_shared<RectificationRemapT>(makeRectificationRemap(config_, calibration_.left, device_info_))),
    right_remap_(std::make_shared<RectificationRemapT>(makeRectificationRemap(config_, calibration_.right, device_info_)))
{
}

void StereoCalibrationManger::updateConfig(const crl::multisense::image::Config& config)
{
    //
    // Only update the calibration if the resolution changed.

    if (config_.width() == config.width() && config_.height() == config.height() && config_.cameraProfile() == config.cameraProfile())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        return;
    }

    auto q_matrix = makeQ(config, calibration_, device_info_);
    auto left_camera_info = makeCameraInfo(config, calibration_.left, compute_scale(config, device_info_));
    auto right_camera_info = makeCameraInfo(config, calibration_.right, compute_scale(config, device_info_));

    const ScaleT aux_scale = config.cameraProfile() == crl::multisense::Full_Res_Aux_Cam ?
                             ScaleT{1., 1., 0., 0.} : compute_scale(config, device_info_);

    auto aux_camera_info = makeCameraInfo(config, calibration_.aux, aux_scale);
    auto left_remap = std::make_shared<RectificationRemapT>(makeRectificationRemap(config, calibration_.left, device_info_));
    auto right_remap = std::make_shared<RectificationRemapT>(makeRectificationRemap(config, calibration_.right, device_info_));

    std::lock_guard<std::mutex> lock(mutex_);

    config_ = config;
    q_matrix_ = std::move(q_matrix);
    left_camera_info_ = std::move(left_camera_info);
    right_camera_info_ = std::move(right_camera_info);
    aux_camera_info_ = std::move(aux_camera_info);
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

Eigen::Vector3d StereoCalibrationManger::aux_T() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    //
    // The aux camera projection matrix is of the form:
    //
    // [fx,  0, cx, tx * fx]
    // [ 0, fy, cy, ty * fy]
    // [ 0,  0,  1, tz     ]
    //
    // divide the t * fx term by fx to get t

    return Eigen::Vector3d{aux_camera_info_.P[3] / aux_camera_info_.P[0],
                           aux_camera_info_.P[7] / aux_camera_info_.P[5],
                           aux_camera_info_.P[11]};
}

Eigen::Vector3f StereoCalibrationManger::reproject(size_t u, size_t v, double d)
{
    if (d == 0.0)
    {
        return Eigen::Vector3f{std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max(),
                               std::numeric_limits<float>::max()};
    }

    std::lock_guard<std::mutex> lock(mutex_);

    const double &fx = left_camera_info_.P[0];
    const double &fy = left_camera_info_.P[5];
    const double &cx = left_camera_info_.P[2];
    const double &cy = left_camera_info_.P[6];
    const double &cx_right = right_camera_info_.P[2];
    const double tx = right_camera_info_.P[3] / right_camera_info_.P[0];

    const double xB = ((fy * tx) * u) + (-fy * cx * tx);
    const double yB = ((fx * tx) * v) + (-fx * cy * tx);
    const double zB = (fx * fy * tx);
    const double invB = 1. / (-fy * d) + (fy * (cx - cx_right));

    return Eigen::Vector3f{static_cast<float>(xB * invB), static_cast<float>(yB * invB), static_cast<float>(zB * invB)};
}

Eigen::Vector2f StereoCalibrationManger::rectifiedAuxProject(const Eigen::Vector3f &left_rectified_point)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const double &fx = aux_camera_info_.P[0];
    const double &fy = aux_camera_info_.P[5];
    const double &cx = aux_camera_info_.P[2];
    const double &cy = aux_camera_info_.P[6];
    const double &fxtx = aux_camera_info_.P[3];
    const double &fyty = aux_camera_info_.P[7];
    const double &tz = aux_camera_info_.P[11];

    //
    // Project the left_rectified_point into the aux image using the auxP matrix
    //
    const double uB = (fx * left_rectified_point(0) + (cx * left_rectified_point(2)) + fxtx);
    const double vB = (fy * left_rectified_point(1) + (cy * left_rectified_point(2)) + fyty);
    const double invB = 1.0 / (left_rectified_point(2) + tz);

    return Eigen::Vector2f{static_cast<float>(uB * invB), static_cast<float>(vB * invB)};
}

OperatingResolutionT StereoCalibrationManger::operatingStereoResolution() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return OperatingResolutionT{config_.width(), config_.height()};
}

OperatingResolutionT StereoCalibrationManger::operatingAuxResolution() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (config_.cameraProfile() == crl::multisense::Full_Res_Aux_Cam)
    {
        return OperatingResolutionT{S30_AUX_CAM_WIDTH, S30_AUX_CAM_HEIGHT};
    }

    const auto scale = compute_scale(config_, device_info_);

    return OperatingResolutionT{config_.width(), static_cast<size_t>(S30_AUX_CAM_HEIGHT * scale.y_scale)};

}

bool StereoCalibrationManger::validAux() const
{
    const Eigen::Vector3d auxT = aux_T();
    return std::isfinite(auxT(0)) && std::isfinite(auxT(1)) && std::isfinite(auxT(2)) ;
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
                                                               const ros::Time& stamp,
                                                               const OperatingResolutionT &resolution) const
{
    return auxCameraInfo(frame_id, stamp, resolution.width, resolution.height);
}

sensor_msgs::CameraInfo StereoCalibrationManger::auxCameraInfo(const std::string& frame_id,
                                                               const ros::Time& stamp,
                                                               size_t width,
                                                               size_t height) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto camera_info = aux_camera_info_;
    camera_info.header.frame_id = frame_id;
    camera_info.header.stamp = stamp;
    camera_info.width = width;
    camera_info.height = height;

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
