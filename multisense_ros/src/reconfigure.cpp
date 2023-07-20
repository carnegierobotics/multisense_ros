/**
 * @file reconfigure.cpp
 *
 * Copyright 2013
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

#include <multisense_ros/reconfigure.h>

using namespace crl::multisense;

namespace multisense_ros {

Reconfigure::Reconfigure(Channel* driver,
                         std::function<void (crl::multisense::image::Config)> resolutionChangeCallback,
                         std::function<void (BorderClip, double)> borderClipChangeCallback,
                         std::function<void (double)> maxPointCloudRangeCallback,
                         std::function<void (crl::multisense::system::ExternalCalibration)> extrinsicsCallback,
                         std::function<void (ground_surface_utilities::SplineDrawParameters)> groundSurfaceSplineDrawParametersCallback):
    driver_(driver),
    resolution_change_callback_(resolutionChangeCallback),
    device_nh_(""),
    imu_samples_per_message_(0),
    lighting_supported_(false),
    motor_supported_(false),
    crop_mode_changed_(false),
    ptp_supported_(false),
    roi_supported_(false),
    aux_supported_(false),
    reconfigure_external_calibration_supported_(false),
    origin_from_camera_calibration_initialized_(false),
    border_clip_type_(BorderClip::NONE),
    border_clip_value_(0.0),
    border_clip_change_callback_(borderClipChangeCallback),
    max_point_cloud_range_callback_(maxPointCloudRangeCallback),
    extrinsics_callback_(extrinsicsCallback),
    spline_draw_parameters_callback_(groundSurfaceSplineDrawParametersCallback)
{
    system::DeviceInfo  deviceInfo;
    system::VersionInfo versionInfo;
    std::vector<system::DeviceMode> deviceModes;

    //
    // Query device and version information from sensor

    Status status = driver_->getVersionInfo(versionInfo);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to query version info: %s",
                  Channel::statusString(status));
        return;
    }

    status = driver_->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    status = driver_->getDeviceModes(deviceModes);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to query device modes: %s",
                  Channel::statusString(status));
        return;
    }

    const bool ground_surface_supported =
        std::any_of(deviceModes.begin(), deviceModes.end(), [](const auto &mode) {
            return (mode.supportedDataSources & Source_Ground_Surface_Spline_Data) &&
                   (mode.supportedDataSources & Source_Ground_Surface_Class_Image); });

    if (deviceInfo.lightingType != 0 ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == deviceInfo.hardwareRevision)
    {
        lighting_supported_ = true;
    }
    if (deviceInfo.motorType != 0)
    {
        motor_supported_ = true;
    }
    if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM == deviceInfo.hardwareRevision)
    {
        ptp_supported_ = true;
    }

    if (versionInfo.sensorFirmwareVersion >= 0x0403) {
        roi_supported_ = true;
    }

    if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == deviceInfo.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == deviceInfo.hardwareRevision)
    {
        if (versionInfo.sensorFirmwareVersion >= 0x0600) {
            aux_supported_ = true;
        }
        else
        {
            aux_supported_ = false;
            ROS_WARN("Reconfigure: MultiSense firmware version does not support the updated aux camera exposure controls. "
                     "The ROS driver will work normally, but you will have limited control over aux camera exposure parameters. "
                      "Please use the 2eae444 has of multisene_ros or contact support@carnegierobotics.com for "
                      "a updated firmware version greater than 6.0 to enable aux camera exposure controls.");
        }
    }

    if (versionInfo.sensorFirmwareVersion >= 0x0513) {
        reconfigure_external_calibration_supported_ = true;
    }

    //
    // Launch the correct reconfigure server for this device configuration.

    if (system::DeviceInfo::HARDWARE_REV_BCAM == deviceInfo.hardwareRevision) {

        server_bcam_imx104_ =
            std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config> > (
                new dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config>(device_nh_));
        server_bcam_imx104_->setCallback(std::bind(&Reconfigure::callback_bcam_imx104, this,
                                                   std::placeholders::_1, std::placeholders::_2));

    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 == deviceInfo.hardwareRevision) {

        server_st21_vga_ =
            std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::st21_sgm_vga_imuConfig> > (
                new dynamic_reconfigure::Server<multisense_ros::st21_sgm_vga_imuConfig>(device_nh_));
        server_st21_vga_->setCallback(std::bind(&Reconfigure::callback_st21_vga, this,
                                                std::placeholders::_1, std::placeholders::_2));

    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST25 == deviceInfo.hardwareRevision) {

        server_st25_sgm_ =
            std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::st25_sgm_imuConfig> > (
                new dynamic_reconfigure::Server<multisense_ros::st25_sgm_imuConfig>(device_nh_));
        server_st25_sgm_->setCallback(std::bind(&Reconfigure::callback_st25_sgm, this,
                                                std::placeholders::_1, std::placeholders::_2));

    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_M == deviceInfo.hardwareRevision) {

        switch(deviceInfo.imagerType) {
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:

            server_mono_cmv2000_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv2000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::mono_cmv2000Config>(device_nh_));
            server_mono_cmv2000_->setCallback(std::bind(&Reconfigure::callback_mono_cmv2000, this,
                                                        std::placeholders::_1, std::placeholders::_2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_mono_cmv4000_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv4000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::mono_cmv4000Config>(device_nh_));
            server_mono_cmv4000_->setCallback(std::bind(&Reconfigure::callback_mono_cmv4000, this,
                                                        std::placeholders::_1, std::placeholders::_2));

            break;
        }
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == deviceInfo.hardwareRevision ||
               system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == deviceInfo.hardwareRevision) {

        if (ground_surface_supported) {
            server_s27_AR0234_ground_surface_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234_ground_surfaceConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234_ground_surfaceConfig>(device_nh_));
            server_s27_AR0234_ground_surface_->setCallback(std::bind(&Reconfigure::callback_s27_AR0234_ground_surface, this,
                                            std::placeholders::_1, std::placeholders::_2));
        } else {
            server_s27_AR0234_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234Config>(device_nh_));
            server_s27_AR0234_->setCallback(std::bind(&Reconfigure::callback_s27_AR0234, this,
                                            std::placeholders::_1, std::placeholders::_2));
        }
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 == deviceInfo.hardwareRevision) {

        if (ground_surface_supported) {
            server_ks21_sgm_AR0234_ground_surface_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234_ground_surfaceConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234_ground_surfaceConfig>(device_nh_));
            server_ks21_sgm_AR0234_ground_surface_->setCallback(std::bind(&Reconfigure::callback_ks21_AR0234_ground_surface, this,
                                                std::placeholders::_1, std::placeholders::_2));
        } else {
            server_ks21_sgm_AR0234_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234Config>(device_nh_));
            server_ks21_sgm_AR0234_->setCallback(std::bind(&Reconfigure::callback_ks21_AR0234, this,
                                                std::placeholders::_1, std::placeholders::_2));
        }
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == deviceInfo.hardwareRevision) {

        if (ground_surface_supported) {
            server_ks21i_sgm_AR0234_ground_surface_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234_ground_surfaceConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234_ground_surfaceConfig>(device_nh_));
            server_ks21i_sgm_AR0234_ground_surface_->setCallback(std::bind(&Reconfigure::callback_ks21i_AR0234_ground_surface, this,
                                                std::placeholders::_1, std::placeholders::_2));
        } else {
            server_ks21i_sgm_AR0234_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234Config>(device_nh_));
            server_ks21i_sgm_AR0234_->setCallback(std::bind(&Reconfigure::callback_ks21i_AR0234, this,
                                                std::placeholders::_1, std::placeholders::_2));
        }
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB == deviceInfo.hardwareRevision) {
        server_remote_head_vpb_ =
            std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_vpbConfig > > (
                new dynamic_reconfigure::Server<multisense_ros::remote_head_vpbConfig>(device_nh_));
        server_remote_head_vpb_->setCallback(std::bind(&Reconfigure::callback_remote_head_vpb, this,
                                                       std::placeholders::_1, std::placeholders::_2));
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO == deviceInfo.hardwareRevision) {
        if (ground_surface_supported) {
            server_remote_head_sgm_AR0234_ground_surface_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234_ground_surfaceConfig > > (
                    new dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234_ground_surfaceConfig>(device_nh_));
            server_remote_head_sgm_AR0234_ground_surface_->setCallback(std::bind(&Reconfigure::callback_remote_head_sgm_AR0234_ground_surface, this,
                                                                                 std::placeholders::_1, std::placeholders::_2));
        } else {
            server_remote_head_sgm_AR0234_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234Config > > (
                    new dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234Config>(device_nh_));
            server_remote_head_sgm_AR0234_->setCallback(std::bind(&Reconfigure::callback_remote_head_sgm_AR0234, this,
                                                                  std::placeholders::_1, std::placeholders::_2));
        }
    } else if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM == deviceInfo.hardwareRevision) {
        server_remote_head_monocam_AR0234_ =
            std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_monocam_AR0234Config > > (
                new dynamic_reconfigure::Server<multisense_ros::remote_head_monocam_AR0234Config>(device_nh_));
        server_remote_head_monocam_AR0234_->setCallback(std::bind(&Reconfigure::callback_remote_head_monocam_AR0234, this,
                                                                  std::placeholders::_1, std::placeholders::_2));
    } else if (versionInfo.sensorFirmwareVersion <= 0x0202) {

        switch(deviceInfo.imagerType) {
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:

            server_sl_bm_cmv2000_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config>(device_nh_));
            server_sl_bm_cmv2000_->setCallback(std::bind(&Reconfigure::callback_sl_bm_cmv2000, this,
                                                         std::placeholders::_1, std::placeholders::_2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_bm_cmv4000_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config>(device_nh_));
            server_sl_bm_cmv4000_->setCallback(std::bind(&Reconfigure::callback_sl_bm_cmv4000, this,
                                                         std::placeholders::_1, std::placeholders::_2));

            break;
        default:

            ROS_ERROR("Reconfigure: unsupported imager type \"%d\"", deviceInfo.imagerType);
            return;
        }

    } else if (versionInfo.sensorFirmwareVersion < 0x0300) {

        switch(deviceInfo.imagerType) {
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:

            server_sl_bm_cmv2000_imu_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig>(device_nh_));
            server_sl_bm_cmv2000_imu_->setCallback(std::bind(&Reconfigure::callback_sl_bm_cmv2000_imu, this,
                                                             std::placeholders::_1, std::placeholders::_2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_bm_cmv4000_imu_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig>(device_nh_));
            server_sl_bm_cmv4000_imu_->setCallback(std::bind(&Reconfigure::callback_sl_bm_cmv4000_imu, this,
                                                             std::placeholders::_1, std::placeholders::_2));

            break;
        default:

            ROS_ERROR("Reconfigure: unsupported imager type \"%d\"", deviceInfo.imagerType);
            return;
        }

    } else {

        switch(deviceInfo.imagerType) {
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:

            server_sl_sgm_cmv2000_imu_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig>(device_nh_));
            server_sl_sgm_cmv2000_imu_->setCallback(std::bind(&Reconfigure::callback_sl_sgm_cmv2000_imu, this,
                                                              std::placeholders::_1, std::placeholders::_2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_sgm_cmv4000_imu_ =
                std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig>(device_nh_));
            server_sl_sgm_cmv4000_imu_->setCallback(std::bind(&Reconfigure::callback_sl_sgm_cmv4000_imu, this,
                                                              std::placeholders::_1, std::placeholders::_2));
            break;
        case system::DeviceInfo::IMAGER_TYPE_AR0234_GREY:
        case system::DeviceInfo::IMAGER_TYPE_AR0239_COLOR:

            if (ground_surface_supported) {
                server_s27_AR0234_ground_surface_ =
                    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234_ground_surfaceConfig> > (
                        new dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234_ground_surfaceConfig>(device_nh_));
                server_s27_AR0234_ground_surface_->setCallback(std::bind(&Reconfigure::callback_s27_AR0234_ground_surface, this,
                                                std::placeholders::_1, std::placeholders::_2));
            } else {
                server_s27_AR0234_ =
                    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234Config> > (
                        new dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234Config>(device_nh_));
                server_s27_AR0234_->setCallback(std::bind(&Reconfigure::callback_s27_AR0234, this,
                                                std::placeholders::_1, std::placeholders::_2));
            }

            break;

        default:

            ROS_ERROR("Reconfigure: unsupported imager type \"%d\"", deviceInfo.imagerType);
            return;
        }
    }

    calibration_ = crl::multisense::system::ExternalCalibration{};
}

Reconfigure::~Reconfigure()
{
}

//
// Helper to change resolution. Will check against supported device modes

bool Reconfigure::changeResolution(image::Config& cfg,
                                   int32_t        width,
                                   int32_t        height,
                                   int32_t        disparities)
{
    //
    // See if we need to change resolutions
    if (width       == static_cast<int32_t>(cfg.width())   &&
        height      == static_cast<int32_t>(cfg.height())  &&
        disparities == static_cast<int32_t>(cfg.disparities()))
        return false;

    //
    // Query all supported resolutions from the sensor, if we haven't already

    if (device_modes_.empty()) {

        Status status = driver_->getDeviceModes(device_modes_);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to query sensor modes: %s",
                      Channel::statusString(status));
            return false;
        }
    }

    //
    // Verify that this resolution is supported

    bool supported = false;
    std::vector<system::DeviceMode>::const_iterator it = device_modes_.begin();
    for(; it != device_modes_.end(); ++it) {

        const system::DeviceMode& m = *it;

        if (width       == static_cast<int32_t>(m.width)  &&
            height      == static_cast<int32_t>(m.height) &&
            disparities == static_cast<int32_t>(m.disparities)) {

            supported = true;
            break;
        }
    }

    if (false == supported) {
        ROS_ERROR("Reconfigure: sensor does not support a resolution of: %dx%d (%d disparities)",
                  width, height, disparities);
        return false;
    }

    ROS_WARN("Reconfigure: changing sensor resolution to %dx%d (%d disparities), from %dx%d "
         "(%d disparities): reconfiguration may take up to 30 seconds",
             width, height, disparities,
             cfg.width(), cfg.height(), cfg.disparities());

    cfg.setResolution(width, height);
    cfg.setDisparities(disparities);

    return true;
}

template<class T> void Reconfigure::configureSgm(image::Config& cfg, const T& dyn)
{
    cfg.setStereoPostFilterStrength(dyn.stereo_post_filtering);
}

template<class T> void Reconfigure::configureHdr(image::Config& cfg, const T& dyn)
{
    cfg.setHdr(dyn.hdr_enable);
}

template<class T> void Reconfigure::configureAutoWhiteBalance(image::Config& cfg, const T& dyn)
{
    cfg.setWhiteBalance(dyn.white_balance_red, dyn.white_balance_blue);
    cfg.setAutoWhiteBalance(dyn.auto_white_balance);
    cfg.setAutoWhiteBalanceDecay(dyn.auto_white_balance_decay);
    cfg.setAutoWhiteBalanceThresh(dyn.auto_white_balance_thresh);
}

template<class T> void Reconfigure::configureGamma(image::Config& cfg, const T& dyn)
{
    cfg.setGamma(dyn.gamma);
}

template<class T> void Reconfigure::configureAuxCamera(const T& dyn)
{
    if (aux_supported_) {

        image::AuxConfig auxConfig;
        Status status = driver_->getAuxImageConfig(auxConfig);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to query aux image config: %s",
                      Channel::statusString(status));
            return;
        }

        //
        // See if we already have a secondary exposure we want to modify. If not create one for the aux camera

        auxConfig.setGain(dyn.aux_gain);
        auxConfig.setGamma(dyn.aux_gamma);
        auxConfig.setExposure(dyn.aux_exposure_time * 1e6);
        auxConfig.setAutoExposure(dyn.aux_auto_exposure);
        auxConfig.setAutoExposureMax(dyn.aux_auto_exposure_max_time * 1e6);
        auxConfig.setAutoExposureDecay(dyn.aux_auto_exposure_decay);
        auxConfig.setAutoExposureThresh(dyn.aux_auto_exposure_thresh);
        auxConfig.setAutoExposureTargetIntensity(dyn.aux_auto_exposure_target_intensity);
        auxConfig.enableSharpening(dyn.aux_enable_sharpening);
        auxConfig.setSharpeningPercentage(dyn.aux_sharpening_percentage);
        auxConfig.setSharpeningLimit(dyn.aux_sharpening_limit);

        auxConfig.setWhiteBalance(dyn.aux_white_balance_red, dyn.aux_white_balance_blue);
        auxConfig.setAutoWhiteBalance(dyn.aux_auto_white_balance);
        auxConfig.setAutoWhiteBalanceDecay(dyn.aux_auto_white_balance_decay);
        auxConfig.setAutoWhiteBalanceThresh(dyn.aux_auto_white_balance_thresh);


        if (dyn.aux_roi_auto_exposure) {
            if (roi_supported_) {
                //
                // Ensure the commanded ROI region is in the full resolution image

                const int32_t maxX = dyn.__getMax__().aux_roi_auto_exposure_x;
                const int32_t maxY = dyn.__getMax__().aux_roi_auto_exposure_y;

                const int32_t x = fmax(0, fmin(dyn.aux_roi_auto_exposure_x, maxX));
                const int32_t y = fmax(0, fmin(dyn.aux_roi_auto_exposure_y, maxY));

               auxConfig.setAutoExposureRoi(x, y,
                                            fmax(0, fmin(dyn.aux_roi_auto_exposure_width, maxX - x)),
                                            fmax(0, fmin(dyn.aux_roi_auto_exposure_height, maxY - y)));
            } else {
                ROS_WARN("Reconfigure: ROI auto exposure is not supported with this firmware version");
                return;
            }

        } else {
            auxConfig.setAutoExposureRoi(0, 0, Roi_Full_Image, Roi_Full_Image);
        }

        status = driver_->setAuxImageConfig(auxConfig);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to set aux image config: %s",
                      Channel::statusString(status));

    }
}

template<class T> void Reconfigure::configureCamera(image::Config& cfg, const T& dyn)
{
    DataSource    streamsEnabled = 0;
    int32_t       width, height, disparities;
    bool          resolutionChange=false;
    Status        status=Status_Ok;

    //
    // Decode the resolution string

    if (3 != sscanf(dyn.resolution.c_str(), "%dx%dx%d", &width, &height, &disparities)) {
        ROS_ERROR("Reconfigure: malformed resolution string: \"%s\"", dyn.resolution.c_str());
        return;
    }

    //
    // If a resolution change is desired

    if ((resolutionChange = changeResolution(cfg, width, height, disparities) || crop_mode_changed_)) {
        crop_mode_changed_ = false;
        //
        // Halt streams during the resolution change
        status = driver_->getEnabledStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to get enabled streams: %s",
                      Channel::statusString(status));
            return;
        }
        status = driver_->stopStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to stop streams for a resolution change: %s",
                      Channel::statusString(status));
            return;
        }
    }

    //
    // Set all other image config from dynamic reconfigure

    cfg.setFps(dyn.fps);
    cfg.setGain(dyn.gain);
    cfg.setExposure(dyn.exposure_time * 1e6);
    cfg.setAutoExposure(dyn.auto_exposure);
    cfg.setAutoExposureMax(dyn.auto_exposure_max_time * 1e6);
    cfg.setAutoExposureDecay(dyn.auto_exposure_decay);
    cfg.setAutoExposureThresh(dyn.auto_exposure_thresh);
    cfg.setAutoExposureTargetIntensity(dyn.auto_exposure_target_intensity);

    if (dyn.roi_auto_exposure) {
        if (roi_supported_) {
            //
            // Ensure the commanded ROI region is in the full resolution image

            const int32_t maxX = dyn.__getMax__().roi_auto_exposure_x;
            const int32_t maxY = dyn.__getMax__().roi_auto_exposure_y;

            const int32_t x = fmax(0, fmin(dyn.roi_auto_exposure_x, maxX));
            const int32_t y = fmax(0, fmin(dyn.roi_auto_exposure_y, maxY));

            cfg.setAutoExposureRoi(x, y,
                                   fmax(0, fmin(dyn.roi_auto_exposure_width, maxX - x)),
                                   fmax(0, fmin(dyn.roi_auto_exposure_height, maxY - y)));
        } else {
            ROS_WARN("Reconfigure: ROI auto exposure is not supported with this firmware version");
        }

    } else {
        cfg.setAutoExposureRoi(0, 0, Roi_Full_Image, Roi_Full_Image);
    }

    //
    // Apply, sensor enforces limits per setting.

    status = driver_->setImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to set image config: %s",
                  Channel::statusString(status));

    status = driver_->getImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to query image config: %s",
                  Channel::statusString(status));

    resolution_change_callback_(cfg);

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {
        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }

    //
    // Enable/disable network-based time synchronization.
    //
    // If enabled, sensor timestamps will be reported in the local
    // system clock's frame, using a continuously updated offset from
    // the sensor's internal clock.
    //
    // If disabled, sensor timestamps will be reported in the sensor
    // clock's frame, which is free-running from zero on power up.
    //
    // Enabled by default.

    driver_->networkTimeSynchronization(dyn.network_time_sync);

    //
    // Set our transmit delay
    image::TransmitDelay d;
    d.delay = dyn.desired_transmit_delay;
    status = driver_->setTransmitDelay(d);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to set transmit delay: %s",
                  Channel::statusString(status));
    }
}

template<class T> void Reconfigure::configureMotor(const T& dyn)
{
    //
    // Send the desired motor speed

    if (motor_supported_) {

        const float radiansPerSecondToRpm = 9.54929659643;

        Status status = driver_->setMotorSpeed(radiansPerSecondToRpm * dyn.motor_speed);
        if (Status_Ok != status) {
            if (Status_Unsupported == status)
                motor_supported_ = false;
            else
                ROS_ERROR("Reconfigure: failed to set motor speed: %s",
                          Channel::statusString(status));
        }
    }
}

template<class T> void Reconfigure::configureLeds(const T& dyn)
{
    //
    // Send the desired lighting configuration

    if (lighting_supported_) {

        lighting::Config leds;

        if (false == dyn.lighting) {
            leds.setFlash(false);
            leds.setDutyCycle(0.0);
        } else {
            leds.setFlash(dyn.flash);
            leds.setDutyCycle(dyn.led_duty_cycle * 100.0);
        }

        Status status = driver_->setLightingConfig(leds);
        if (Status_Ok != status) {
            if (Status_Unsupported == status)
                lighting_supported_ = false;
            else
                ROS_ERROR("Reconfigure: failed to set lighting config: %s",
                          Channel::statusString(status));
        }
    }
}

template<class T> void Reconfigure::configureS19Leds(const T& dyn)
{
    //
    // Send the desired lighting configuration

    if (lighting_supported_) {

        lighting::Config leds;

        if (false == dyn.lighting) {
            leds.setFlash(false);
            leds.setDutyCycle(0.0);
        } else {
            leds.setFlash(dyn.flash);
            leds.setDutyCycle(dyn.led_duty_cycle * 100.0);
            leds.setNumberOfPulses(dyn.led_number_of_pulses);
            leds.setStartupTime(dyn.led_startup_time_us);
            leds.setInvertPulse(dyn.led_invert_pulse);
        }

        Status status = driver_->setLightingConfig(leds);
        if (Status_Ok != status) {
            if (Status_Unsupported == status)
                lighting_supported_ = false;
            else
                ROS_ERROR("Reconfigure: failed to set lighting config: %s",
                          Channel::statusString(status));
        }
    }
}

template<class T> void Reconfigure::configureImu(const T& dyn)
{
    if (imu_configs_.empty()) {
        Status status = driver_->getImuConfig(imu_samples_per_message_,
                                              imu_configs_);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to query IMU config: %s",
                      Channel::statusString(status));
            return;
        }
    }

    std::vector<imu::Config> changedConfigs;
    std::vector<imu::Config>::iterator it = imu_configs_.begin();
    for(; it!=imu_configs_.end(); ++it) {

        imu::Config& c = *it;

        if ("accelerometer" == c.name &&
            (c.enabled      != dyn.accelerometer_enabled ||
             static_cast<int>(c.rateTableIndex)  != dyn.accelerometer_rate    ||
             static_cast<int>(c.rangeTableIndex) != dyn.accelerometer_range)) {

            c.enabled         = dyn.accelerometer_enabled;
            c.rateTableIndex  = dyn.accelerometer_rate;
            c.rangeTableIndex = dyn.accelerometer_range;
            changedConfigs.push_back(c);
        }

        if ("gyroscope" == c.name &&
            (c.enabled  != dyn.gyroscope_enabled ||
             static_cast<int>(c.rateTableIndex)  != dyn.gyroscope_rate    ||
             static_cast<int>(c.rangeTableIndex) != dyn.gyroscope_range)) {

            c.enabled         = dyn.gyroscope_enabled;
            c.rateTableIndex  = dyn.gyroscope_rate;
            c.rangeTableIndex = dyn.gyroscope_range;
            changedConfigs.push_back(c);
        }

        if ("magnetometer" == c.name &&
            (c.enabled     != dyn.magnetometer_enabled ||
             static_cast<int>(c.rateTableIndex)  != dyn.magnetometer_rate    ||
             static_cast<int>(c.rangeTableIndex) != dyn.magnetometer_range)) {

            c.enabled         = dyn.magnetometer_enabled;
            c.rateTableIndex  = dyn.magnetometer_rate;
            c.rangeTableIndex = dyn.magnetometer_range;
            changedConfigs.push_back(c);
        }
    }

    if (changedConfigs.size() > 0 ||
        static_cast<int>(imu_samples_per_message_) != dyn.imu_samples_per_message) {

        ROS_WARN("Reconfigure: IMU configuration changes will take effect after all IMU "
         "topic subscriptions have been closed.");

        imu_samples_per_message_ = dyn.imu_samples_per_message;

        Status status = driver_->setImuConfig(false, // store in non-volatile flash
                                              imu_samples_per_message_,
                                              changedConfigs);  // can be empty
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to set IMU configuration: %s",
                      Channel::statusString(status));
            imu_configs_.clear();
        }
    }
}

template<class T> void Reconfigure::configureBorderClip(const T& dyn)
{

    if (static_cast<BorderClip>(dyn.border_clip_type) != border_clip_type_ ||
        dyn.border_clip_value != border_clip_value_) {
        border_clip_type_ = static_cast<BorderClip>(dyn.border_clip_type);
        border_clip_value_ = dyn.border_clip_value;
        border_clip_change_callback_(border_clip_type_, border_clip_value_);
    }
}

template<class T> void Reconfigure::configurePointCloudRange(const T& dyn)
{
    max_point_cloud_range_callback_(dyn.max_point_cloud_range);
}

template<class T> void Reconfigure::configurePtp(const T& dyn)
{
    if (ptp_supported_) {
        Status status = driver_->ptpTimeSynchronization(dyn.ptp_time_sync);
        if (Status_Ok != status) {
            if (Status_Unsupported == status || Status_Unknown == status) {
                ptp_supported_ = false;
            } else {
                ROS_ERROR("Reconfigure: enable PTP time synchronization: %s",
                          Channel::statusString(status));
            }
        }
    }

    if (dyn.trigger_source != 3 ||  (ptp_supported_ && dyn.trigger_source == 3)) {
        Status status = driver_->setTriggerSource(dyn.trigger_source);
        if (Status_Ok != status) {
            if (Status_Unsupported == status || Status_Unknown == status) {
                ptp_supported_ = false;
            } else {
                ROS_ERROR("Reconfigure: failed to set trigger source: %s",
                          Channel::statusString(status));
            }
        }
    }
}

template<class T> void Reconfigure::configureStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn)
{
    profile |= (dyn.high_contrast_profile ? crl::multisense::High_Contrast : profile);
    profile |= (dyn.show_roi_profile ? crl::multisense::Show_ROIs : profile);
}

template<class T> void Reconfigure::configureGroundSurfaceStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn)
{
    profile |= (dyn.ground_surface_profile ? crl::multisense::Ground_Surface : profile);
}

template<class T> void Reconfigure::configureFullResAuxStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn)
{
    profile |= (dyn.full_res_aux_profile ? crl::multisense::Full_Res_Aux_Cam : profile);
}

template<class T> void Reconfigure::configureDetailDisparityStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn)
{
    profile |= (dyn.detail_disparity_profile ? crl::multisense::Detail_Disparity : profile);
}

template<class T> void Reconfigure::configureExtrinsics(const T& dyn)
{
    if (!dyn.enable_origin_from_camera_configuration)
        return;

    //
    // Setup extrinsics transform tree
    if (!origin_from_camera_calibration_initialized_)
    {
        extrinsics_callback_(calibration_);

        origin_from_camera_calibration_initialized_ = true;

        return;
    }

    //
    // If supported, reconfigure with new dynamic configuration values
    if (!reconfigure_external_calibration_supported_)
        return;

    constexpr float deg_to_rad = M_PI / 180.0f;
    if (std::abs(dyn.origin_from_camera_position_x_m - calibration_.x) < 1e-3 &&
        std::abs(dyn.origin_from_camera_position_y_m - calibration_.y) < 1e-3 &&
        std::abs(dyn.origin_from_camera_position_z_m - calibration_.z) < 1e-3 &&
        std::abs(dyn.origin_from_camera_rotation_x_deg * deg_to_rad - calibration_.roll) < 1e-3 &&
        std::abs(dyn.origin_from_camera_rotation_y_deg * deg_to_rad - calibration_.pitch) < 1e-3 &&
        std::abs(dyn.origin_from_camera_rotation_z_deg * deg_to_rad - calibration_.yaw) < 1e-3) {
        return;
    }

    //
    // Update calibration on camera via libmultisense

    calibration_.x = dyn.origin_from_camera_position_x_m;
    calibration_.y = dyn.origin_from_camera_position_y_m;
    calibration_.z = dyn.origin_from_camera_position_z_m;

    calibration_.roll = dyn.origin_from_camera_rotation_x_deg * deg_to_rad;
    calibration_.pitch = dyn.origin_from_camera_rotation_y_deg * deg_to_rad;
    calibration_.yaw = dyn.origin_from_camera_rotation_z_deg * deg_to_rad;

    Status status = driver_->setExternalCalibration(calibration_);
    if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to set external calibration: %s",
                        Channel::statusString(status));
        return;
    }

    // Update camera class locally to modify pointcloud transform in rviz
    extrinsics_callback_(calibration_);
}

template<class T> void Reconfigure::configureGroundSurfaceParams(const T& dyn)
{
    //
    // Update calibration on camera via libmultisense
    crl::multisense::system::GroundSurfaceParams params;

    params.ground_surface_number_of_levels_x = dyn.ground_surface_spline_resolution_x;
    params.ground_surface_number_of_levels_z = dyn.ground_surface_spline_resolution_z;

    if (dyn.ground_surface_pre_transform_data == "Quadratic")
        params.ground_surface_base_model = 0;
    else if (dyn.ground_surface_pre_transform_data == "Mean")
        params.ground_surface_base_model = 1;
    else if (dyn.ground_surface_pre_transform_data == "Zero")
        params.ground_surface_base_model = 2;

    params.ground_surface_pointcloud_grid_size = dyn.ground_surface_pointcloud_grid_size;
    params.ground_surface_min_points_per_grid = dyn.ground_surface_min_points_per_grid;
    params.ground_surface_pointcloud_decimation = dyn.ground_surface_pointcloud_decimation;
    params.ground_surface_pointcloud_max_range_m = dyn.ground_surface_pointcloud_global_max_z_m;
    params.ground_surface_pointcloud_min_range_m = dyn.ground_surface_pointcloud_global_min_z_m;
    params.ground_surface_pointcloud_max_width_m = dyn.ground_surface_pointcloud_global_max_x_m;
    params.ground_surface_pointcloud_min_width_m = dyn.ground_surface_pointcloud_global_min_x_m;
    params.ground_surface_pointcloud_max_height_m = dyn.ground_surface_pointcloud_global_max_height_m;
    params.ground_surface_pointcloud_min_height_m = dyn.ground_surface_pointcloud_global_min_height_m;
    params.ground_surface_obstacle_height_thresh_m = dyn.ground_surface_obstacle_height_thresh_m;
    params.ground_surface_obstacle_percentage_thresh = dyn.ground_surface_obstacle_percentage_thresh;
    params.ground_surface_max_fitting_iterations = dyn.ground_surface_max_fitting_iterations;
    params.ground_surface_adjacent_cell_search_size_m = dyn.ground_surface_adjacent_cell_search_size_m;

    // Update ground surface params on camera
    Status status = driver_->setGroundSurfaceParams(params);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to set ground surface params: %s",
                  Channel::statusString(status));
        return;
    }

    //
    // Update spline drawing parameters locally
    spline_draw_parameters_callback_(
        ground_surface_utilities::SplineDrawParameters{
        dyn.ground_surface_pointcloud_global_max_z_m,
        dyn.ground_surface_pointcloud_global_min_z_m,
        dyn.ground_surface_pointcloud_global_max_x_m,
        dyn.ground_surface_pointcloud_global_min_x_m,
        dyn.ground_surface_spline_draw_resolution}
    );
}

template<class T> void Reconfigure::configureRemoteHeadSyncGroups(const T& dyn)
{
    // convert the dyn integer into a remote head channel enum
    RemoteHeadChannel c1 = dyn.sync_group_1_controller != -1
        ?  static_cast<RemoteHeadChannel>(dyn.sync_group_1_controller) : Remote_Head_Invalid;
    RemoteHeadChannel r1 = dyn.sync_group_1_responder != -1
        ?  static_cast<RemoteHeadChannel>(dyn.sync_group_1_responder) : Remote_Head_Invalid;

    RemoteHeadChannel c2 = dyn.sync_group_2_controller != -1
        ?  static_cast<RemoteHeadChannel>(dyn.sync_group_2_controller) : Remote_Head_Invalid;
    RemoteHeadChannel r2 = dyn.sync_group_2_responder != -1
        ?  static_cast<RemoteHeadChannel>(dyn.sync_group_2_responder) : Remote_Head_Invalid;

    const std::vector<RemoteHeadSyncGroup> sync_groups{{c1, {r1}}, {c2, {r2}}};

    const image::RemoteHeadConfig rh_config{sync_groups};

    // Update ground surface params on camera
    Status status = driver_->setRemoteHeadConfig(rh_config);
    if (Status_Ok != status) {
        ROS_ERROR("Reconfigure: failed to set remote head config: %s",
                  Channel::statusString(status));
        return;
    }
}

#define GET_CONFIG()                                                    \
    image::Config cfg;                                                  \
    Status status = driver_->getImageConfig(cfg);                       \
    if (Status_Ok != status) {                                          \
        ROS_ERROR("Reconfigure: failed to query image config: %s",      \
                  Channel::statusString(status));                       \
        return;                                                         \
    }                                                                   \

#define SL_BM()  do {                                           \
        GET_CONFIG();                                           \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureMotor(dyn);                                    \
        configureLeds(dyn);                                     \
        configureBorderClip(dyn);                               \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define SL_BM_IMU()  do {                                       \
        GET_CONFIG();                                           \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureMotor(dyn);                                    \
        configureLeds(dyn);                                     \
        configureImu(dyn);                                      \
        configureBorderClip(dyn);                               \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define MONO_BM_IMU()  do {                                     \
        GET_CONFIG();                                           \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureLeds(dyn);                                     \
        configureImu(dyn);                                      \
        configureExtrinsics(dyn);                               \
    } while(0)

#define SL_SGM_IMU()  do {                                      \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        configureHdr(cfg, dyn);                                 \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureMotor(dyn);                                    \
        configureLeds(dyn);                                     \
        configureImu(dyn);                                      \
        configureBorderClip(dyn);                               \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define SL_SGM()  do {                                          \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        configureHdr(cfg, dyn);                                 \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define SL_SGM_IMU_CMV4000()  do {                              \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        configureHdr(cfg, dyn);                                 \
        configureAutoWhiteBalance(cfg, dyn);                    \
        configureCamera(cfg, dyn);                              \
        configureMotor(dyn);                                    \
        configureLeds(dyn);                                     \
        configureImu(dyn);                                      \
        configureBorderClip(dyn);                               \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define S27_SGM()  do {                                         \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureFullResAuxStereoProfile(profile, dyn);         \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureAuxCamera(dyn);                                \
    } while(0)

#define KS21_SGM()  do {                                        \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define KS21I_SGM()  do {                                        \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureFullResAuxStereoProfile(profile, dyn);         \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureAuxCamera(dyn);                                \
    } while(0)

#define S27_SGM_GROUND_SURFACE()  do {                          \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureGroundSurfaceStereoProfile(profile, dyn);      \
        configureFullResAuxStereoProfile(profile, dyn);         \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureGroundSurfaceParams(dyn);                      \
        configureAuxCamera(dyn);                                \
    } while(0)

#define KS21_SGM_GROUND_SURFACE()  do {                         \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureGroundSurfaceStereoProfile(profile, dyn);      \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureGroundSurfaceParams(dyn);                      \
    } while(0)

#define KS21I_SGM_GROUND_SURFACE()  do {                         \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureGroundSurfaceStereoProfile(profile, dyn);      \
        configureFullResAuxStereoProfile(profile, dyn);         \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureGroundSurfaceParams(dyn);                      \
        configureAuxCamera(dyn);                                \
    } while(0)

#define REMOTE_HEAD_VPB()  do {                                 \
        GET_CONFIG();                                           \
        configurePtp(dyn);                                      \
        configureExtrinsics(dyn);                               \
        configureRemoteHeadSyncGroups(dyn);                     \
    } while(0)

#define REMOTE_HEAD_SGM_AR0234()  do {                          \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
    } while(0)

#define REMOTE_HEAD_SGM_AR0234_GROUND_SURFACE()  do {           \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        configureDetailDisparityStereoProfile(profile, dyn);    \
        configureGroundSurfaceStereoProfile(profile, dyn);      \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureBorderClip(dyn);                               \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configurePointCloudRange(dyn);                          \
        configureExtrinsics(dyn);                               \
        configureGroundSurfaceParams(dyn);                      \
    } while(0)

#define REMOTE_HEAD_MONOCAM_AR0234()  do {                      \
        GET_CONFIG();                                           \
        crl::multisense::CameraProfile profile = crl::multisense::User_Control; \
        configureStereoProfile(profile, dyn);                   \
        cfg.setCameraProfile(profile);                          \
        configureGamma(cfg, dyn);                               \
        configureCamera(cfg, dyn);                              \
        configureS19Leds(dyn);                                  \
        configurePtp(dyn);                                      \
        configureExtrinsics(dyn);                               \
    } while(0)

//
// The dynamic reconfigure callbacks (MultiSense S* & feature variations)

void Reconfigure::callback_sl_bm_cmv2000             (multisense_ros::sl_bm_cmv2000Config&              dyn, uint32_t level) { (void) level; SL_BM(); }
void Reconfigure::callback_sl_bm_cmv2000_imu         (multisense_ros::sl_bm_cmv2000_imuConfig&          dyn, uint32_t level) { (void) level; SL_BM_IMU(); }
void Reconfigure::callback_sl_bm_cmv4000             (multisense_ros::sl_bm_cmv4000Config&              dyn, uint32_t level) { (void) level; SL_BM(); }
void Reconfigure::callback_sl_bm_cmv4000_imu         (multisense_ros::sl_bm_cmv4000_imuConfig&          dyn, uint32_t level) { (void) level; SL_BM_IMU(); }
void Reconfigure::callback_sl_sgm_cmv2000_imu        (multisense_ros::sl_sgm_cmv2000_imuConfig&         dyn, uint32_t level) { (void) level; SL_SGM_IMU(); }
void Reconfigure::callback_sl_sgm_cmv4000_imu        (multisense_ros::sl_sgm_cmv4000_imuConfig&         dyn, uint32_t level) { (void) level; SL_SGM_IMU_CMV4000(); }
void Reconfigure::callback_mono_cmv2000              (multisense_ros::mono_cmv2000Config&               dyn, uint32_t level) { (void) level; MONO_BM_IMU(); }
void Reconfigure::callback_mono_cmv4000              (multisense_ros::mono_cmv4000Config&               dyn, uint32_t level) { (void) level; MONO_BM_IMU(); }
void Reconfigure::callback_s27_AR0234                (multisense_ros::s27_sgm_AR0234Config&             dyn, uint32_t level) { (void) level; S27_SGM(); }
void Reconfigure::callback_ks21_AR0234               (multisense_ros::ks21_sgm_AR0234Config&            dyn, uint32_t level) { (void) level; KS21_SGM(); }
void Reconfigure::callback_ks21i_AR0234              (multisense_ros::ks21i_sgm_AR0234Config&           dyn, uint32_t level) { (void) level; KS21I_SGM(); }
void Reconfigure::callback_remote_head_vpb           (multisense_ros::remote_head_vpbConfig&            dyn, uint32_t level) { (void) level; REMOTE_HEAD_VPB(); }
void Reconfigure::callback_remote_head_sgm_AR0234    (multisense_ros::remote_head_sgm_AR0234Config&     dyn, uint32_t level) { (void) level; REMOTE_HEAD_SGM_AR0234(); }
void Reconfigure::callback_remote_head_monocam_AR0234(multisense_ros::remote_head_monocam_AR0234Config& dyn, uint32_t level) { (void) level; REMOTE_HEAD_MONOCAM_AR0234(); }

void Reconfigure::callback_s27_AR0234_ground_surface            (multisense_ros::s27_sgm_AR0234_ground_surfaceConfig&         dyn, uint32_t level) { (void) level; S27_SGM_GROUND_SURFACE();  }
void Reconfigure::callback_ks21_AR0234_ground_surface           (multisense_ros::ks21_sgm_AR0234_ground_surfaceConfig&        dyn, uint32_t level) { (void) level; KS21_SGM_GROUND_SURFACE(); }
void Reconfigure::callback_ks21i_AR0234_ground_surface          (multisense_ros::ks21i_sgm_AR0234_ground_surfaceConfig&       dyn, uint32_t level) { (void) level; KS21I_SGM_GROUND_SURFACE(); }
void Reconfigure::callback_remote_head_sgm_AR0234_ground_surface(multisense_ros::remote_head_sgm_AR0234_ground_surfaceConfig& dyn, uint32_t level) { (void) level; REMOTE_HEAD_SGM_AR0234_GROUND_SURFACE(); }

//
// BCAM (Sony IMX104)

void Reconfigure::callback_bcam_imx104(multisense_ros::bcam_imx104Config& dyn,
                                       uint32_t                           level)
{
    (void) level;

    GET_CONFIG();
    DataSource  streamsEnabled = 0;
    int32_t     width, height;
    bool        resolutionChange=false;

    //
    // Decode the resolution string

    if (2 != sscanf(dyn.resolution.c_str(), "%dx%dx", &width, &height)) {
        ROS_ERROR("Reconfigure: malformed resolution string: \"%s\"", dyn.resolution.c_str());
        return;
    }

    //
    // If a resolution change is desired

    if ((resolutionChange = changeResolution(cfg, width, height, 0))) {

        //
        // Halt streams during the resolution change

        status = driver_->getEnabledStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to get enabled streams: %s",
                      Channel::statusString(status));
            return;
        }
        status = driver_->stopStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to stop streams for a resolution change: %s",
                      Channel::statusString(status));
            return;
        }
    }

    //
    // Set all other image config from dynamic reconfigure

    cfg.setFps(static_cast<float>(dyn.fps));
    cfg.setGain(dyn.gain);
    cfg.setExposure(dyn.exposure_time * 1e6);
    cfg.setAutoExposure(dyn.auto_exposure);
    cfg.setAutoExposureMax(dyn.auto_exposure_max_time * 1e6);
    cfg.setAutoExposureDecay(dyn.auto_exposure_decay);
    cfg.setAutoExposureThresh(dyn.auto_exposure_thresh);
    cfg.setWhiteBalance(dyn.white_balance_red,
                        dyn.white_balance_blue);
    cfg.setAutoWhiteBalance(dyn.auto_white_balance);
    cfg.setAutoWhiteBalanceDecay(dyn.auto_white_balance_decay);
    cfg.setAutoWhiteBalanceThresh(dyn.auto_white_balance_thresh);

    //
    // Apply, sensor enforces limits per setting.

    status = driver_->setImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to set image config: %s",
                  Channel::statusString(status));

    status = driver_->getImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to query image config: %s",
                  Channel::statusString(status));

    resolution_change_callback_(cfg);

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {

        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }
}

//
// ST21 FLIR Thermal Imagers
// Seperate callback required due to limited subset of dyn parameters
// in st21_sgm_vga_imuConfig. configureCamera results in SFINAE errors

void Reconfigure::callback_st21_vga(multisense_ros::st21_sgm_vga_imuConfig& dyn,
                                       uint32_t                           level)
{
    (void) level;
    DataSource    streamsEnabled = 0;
    int32_t       width, height, disparities;
    bool          resolutionChange=false;

    GET_CONFIG();

    //
    // Decode the resolution string

    if (3 != sscanf(dyn.resolution.c_str(), "%dx%dx%d", &width, &height, &disparities)) {
        ROS_ERROR("Reconfigure: malformed resolution string: \"%s\"", dyn.resolution.c_str());
        return;
    }

    //
    // If a resolution change is desired

    if ((resolutionChange = changeResolution(cfg, width, height, disparities))) {

        //
        // Halt streams during the resolution change

        status = driver_->getEnabledStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to get enabled streams: %s",
                      Channel::statusString(status));
            return;
        }
        status = driver_->stopStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to stop streams for a resolution change: %s",
                      Channel::statusString(status));
            return;
        }
    }

    cfg.setFps(dyn.fps);

    configureSgm(cfg, dyn);
    configureImu(dyn);

    //
    // Apply, sensor enforces limits per setting.

    status = driver_->setImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to set image config: %s",
                  Channel::statusString(status));

    status = driver_->getImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to query image config: %s",
                  Channel::statusString(status));

    resolution_change_callback_(cfg);

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {

        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }

    configureBorderClip(dyn);
    configurePointCloudRange(dyn);
}

//
// ST25 Thermal Imagers
// Seperate callback required due to limited subset of dyn parameters
// in st25_sgm_imuConfig. configureCamera results in SFINAE errors

void Reconfigure::callback_st25_sgm(multisense_ros::st25_sgm_imuConfig& dyn,
                                    uint32_t                            level)
{
    (void) level;
    DataSource    streamsEnabled = 0;
    int32_t       width, height, disparities;
    bool          resolutionChange=false;

    GET_CONFIG();

    //
    // Decode the resolution string

    if (3 != sscanf(dyn.resolution.c_str(), "%dx%dx%d", &width, &height, &disparities)) {
        ROS_ERROR("Reconfigure: malformed resolution string: \"%s\"", dyn.resolution.c_str());
        return;
    }

    //
    // If a resolution change is desired

    if ((resolutionChange = changeResolution(cfg, width, height, disparities))) {

        //
        // Halt streams during the resolution change

        status = driver_->getEnabledStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to get enabled streams: %s",
                      Channel::statusString(status));
            return;
        }
        status = driver_->stopStreams(streamsEnabled);
        if (Status_Ok != status) {
            ROS_ERROR("Reconfigure: failed to stop streams for a resolution change: %s",
                      Channel::statusString(status));
            return;
        }
    }

    cfg.setFps(dyn.fps);

    configureSgm(cfg, dyn);
    configureImu(dyn);

    //
    // Apply, sensor enforces limits per setting.

    status = driver_->setImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to set image config: %s",
                  Channel::statusString(status));

    status = driver_->getImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Reconfigure: failed to query image config: %s",
                  Channel::statusString(status));

    resolution_change_callback_(cfg);

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {

        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }

    configureBorderClip(dyn);
    configurePointCloudRange(dyn);
}

} // namespace
