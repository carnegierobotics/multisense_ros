/**
 * @file reconfigure.h
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

#ifndef MULTISENSE_ROS_RECONFIGURE_H
#define MULTISENSE_ROS_RECONFIGURE_H

#include <ros/ros.h>
#include <chrono>
#include <thread>

#include <multisense_lib/MultiSenseChannel.hh>

#include <dynamic_reconfigure/server.h>
#include <multisense_ros/sl_bm_cmv2000Config.h>
#include <multisense_ros/sl_bm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_bm_cmv4000Config.h>
#include <multisense_ros/sl_bm_cmv4000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv4000_imuConfig.h>
#include <multisense_ros/bcam_imx104Config.h>
#include <multisense_ros/st21_sgm_vga_imuConfig.h>
#include <multisense_ros/st25_sgm_imuConfig.h>
#include <multisense_ros/mono_cmv2000Config.h>
#include <multisense_ros/mono_cmv4000Config.h>
#include <multisense_ros/s27_sgm_AR0234Config.h>
#include <multisense_ros/s27_sgm_AR0234_ground_surfaceConfig.h>
#include <multisense_ros/ks21_sgm_AR0234Config.h>
#include <multisense_ros/ks21_sgm_AR0234_ground_surfaceConfig.h>
#include <multisense_ros/ks21i_sgm_AR0234Config.h>
#include <multisense_ros/ks21i_sgm_AR0234_ground_surfaceConfig.h>
#include <multisense_ros/remote_head_vpbConfig.h>
#include <multisense_ros/remote_head_sgm_AR0234Config.h>
#include <multisense_ros/remote_head_sgm_AR0234_ground_surfaceConfig.h>
#include <multisense_ros/remote_head_monocam_AR0234Config.h>
#include <multisense_ros/camera_utilities.h>
#include <multisense_ros/ground_surface_utilities.h>

namespace multisense_ros {

class Reconfigure {
public:

    Reconfigure(crl::multisense::Channel* driver,
                std::function<void (crl::multisense::image::Config)> resolutionChangeCallback,
                std::function<void (BorderClip, double)> borderClipChangeCallback,
                std::function<void (double)> maxPointCloudRangeCallback,
                std::function<void (crl::multisense::system::ExternalCalibration)> extrinsicsCallback,
                std::function<void (ground_surface_utilities::SplineDrawParameters)> groundSurfaceSplineDrawParametersCallback,
                std::function<void (bool, int32_t)> timeSyncChangedCallback);

    ~Reconfigure();

    void imuCallback(const crl::multisense::imu::Header& header);

private:

    //
    // Dynamic reconfigure callback variations

    void callback_sl_bm_cmv2000     (multisense_ros::sl_bm_cmv2000Config&      config, uint32_t level);
    void callback_sl_bm_cmv2000_imu (multisense_ros::sl_bm_cmv2000_imuConfig&  config, uint32_t level);
    void callback_sl_bm_cmv4000     (multisense_ros::sl_bm_cmv4000Config&      config, uint32_t level);
    void callback_sl_bm_cmv4000_imu (multisense_ros::sl_bm_cmv4000_imuConfig&  config, uint32_t level);
    void callback_sl_sgm_cmv2000_imu(multisense_ros::sl_sgm_cmv2000_imuConfig& config, uint32_t level);
    void callback_sl_sgm_cmv4000_imu(multisense_ros::sl_sgm_cmv4000_imuConfig& config, uint32_t level);
    void callback_bcam_imx104       (multisense_ros::bcam_imx104Config&        config, uint32_t level);
    void callback_st21_vga          (multisense_ros::st21_sgm_vga_imuConfig&   config, uint32_t level);
    void callback_st25_sgm          (multisense_ros::st25_sgm_imuConfig&       config, uint32_t level);
    void callback_mono_cmv2000      (multisense_ros::mono_cmv2000Config&       config, uint32_t level);
    void callback_mono_cmv4000      (multisense_ros::mono_cmv4000Config&       config, uint32_t level);
    void callback_s27_AR0234        (multisense_ros::s27_sgm_AR0234Config&     config, uint32_t level);
    void callback_ks21_AR0234       (multisense_ros::ks21_sgm_AR0234Config&    config, uint32_t level);
    void callback_ks21i_AR0234      (multisense_ros::ks21i_sgm_AR0234Config&   config, uint32_t level);

    void callback_s27_AR0234_ground_surface        (multisense_ros::s27_sgm_AR0234_ground_surfaceConfig&     dyn, uint32_t level);
    void callback_ks21_AR0234_ground_surface       (multisense_ros::ks21_sgm_AR0234_ground_surfaceConfig&    dyn, uint32_t level);
    void callback_ks21i_AR0234_ground_surface      (multisense_ros::ks21i_sgm_AR0234_ground_surfaceConfig&   dyn, uint32_t level);

    void callback_remote_head_vpb                      (multisense_ros::remote_head_vpbConfig&                       dyn, uint32_t level);
    void callback_remote_head_sgm_AR0234               (multisense_ros::remote_head_sgm_AR0234Config&                dyn, uint32_t level);
    void callback_remote_head_sgm_AR0234_ground_surface(multisense_ros::remote_head_sgm_AR0234_ground_surfaceConfig& dyn, uint32_t level);
    void callback_remote_head_monocam_AR0234           (multisense_ros::remote_head_monocam_AR0234Config&            dyn, uint32_t level);

    //
    // Internal helper functions

    bool changeResolution(crl::multisense::image::Config& cfg,
                          int32_t width, int32_t height, int32_t disparities);
    template<class T> void configureSgm(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureHdr(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureAutoWhiteBalance(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureGamma(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureAuxCamera(const T& dyn);
    template<class T> void configureCamera(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureMotor(const T& dyn);
    template<class T> void configureLeds(const T& dyn);
    template<class T> void configureS19Leds(const T& dyn);
    template<class T> void configureImu(const T& dyn);
    template<class T> void configureBorderClip(const T& dyn);
    template<class T> void configurePointCloudRange(const T& dyn);
    template<class T> void configurePtp(const T& dyn);
    template<class T> void configureStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn);
    template<class T> void configureGroundSurfaceStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn);
    template<class T> void configureFullResAuxStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn);
    template<class T> void configureDetailDisparityStereoProfile(crl::multisense::CameraProfile &profile, const T& dyn);
    template<class T> void configureExtrinsics(const T& dyn);
    template<class T> void configureGroundSurfaceParams(const T& dyn);
    template<class T> void configureRemoteHeadSyncGroups(const T& dyn);

    //
    // CRL sensor API

    crl::multisense::Channel* driver_ = nullptr;

    //
    // Resolution change callback

    std::function<void (crl::multisense::image::Config)> resolution_change_callback_;

    //
    // Driver nodes

    ros::NodeHandle device_nh_;

    //
    // Cached modes from the sensor

    std::vector<crl::multisense::system::DeviceMode> device_modes_;
    uint32_t                                         imu_samples_per_message_;
    std::vector<crl::multisense::imu::Config>        imu_configs_;

    //
    // Dynamic reconfigure server variations

    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config> >      server_sl_bm_cmv2000_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig> >  server_sl_bm_cmv2000_imu_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config> >      server_sl_bm_cmv4000_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig> >  server_sl_bm_cmv4000_imu_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig> > server_sl_sgm_cmv2000_imu_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig> > server_sl_sgm_cmv4000_imu_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config> >        server_bcam_imx104_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::st21_sgm_vga_imuConfig> >   server_st21_vga_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::st25_sgm_imuConfig> >       server_st25_sgm_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv2000Config> >       server_mono_cmv2000_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::mono_cmv4000Config> >       server_mono_cmv4000_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234Config> >     server_s27_AR0234_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234Config> >    server_ks21_sgm_AR0234_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234Config> >   server_ks21i_sgm_AR0234_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::s27_sgm_AR0234_ground_surfaceConfig> >     server_s27_AR0234_ground_surface_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21_sgm_AR0234_ground_surfaceConfig> >    server_ks21_sgm_AR0234_ground_surface_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::ks21i_sgm_AR0234_ground_surfaceConfig> >   server_ks21i_sgm_AR0234_ground_surface_;

    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_vpbConfig> > server_remote_head_vpb_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234Config> > server_remote_head_sgm_AR0234_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_sgm_AR0234_ground_surfaceConfig> > server_remote_head_sgm_AR0234_ground_surface_;
    std::shared_ptr< dynamic_reconfigure::Server<multisense_ros::remote_head_monocam_AR0234Config> > server_remote_head_monocam_AR0234_;

    //
    // Cached values for supported sub-systems (these may be unavailable on
    // certain hardware variations: S7, S7S, M, etc.)

    bool lighting_supported_ = false;
    bool motor_supported_ = false;
    bool crop_mode_changed_ = false;
    bool ptp_supported_ = false;
    bool roi_supported_ = false;
    bool aux_supported_ = false;
    bool reconfigure_external_calibration_supported_ = false;
    bool origin_from_camera_calibration_initialized_ = false;

    //
    // Cached value for the boarder clip. These are used to determine if we
    // should regenerate our border clipping mask

    BorderClip border_clip_type_ = BorderClip::NONE;
    double border_clip_value_ = 0.0;

    //
    // Border clip change callback

    std::function<void (BorderClip, double)> border_clip_change_callback_;

    //
    // Max point cloud range callback

    std::function<void (double)> max_point_cloud_range_callback_;

    //
    // Extrinsics callback to modify pointcloud

    crl::multisense::system::ExternalCalibration calibration_;
    std::function<void (crl::multisense::system::ExternalCalibration)> extrinsics_callback_;

    //
    // Extrinsics callback to draw spline parameters

    std::function<void (ground_surface_utilities::SplineDrawParameters)> spline_draw_parameters_callback_;

    //
    // Callback to modify time sync state

    std::function<void (bool, int32_t)> time_sync_callback_;
};

} // multisense_ros

#endif
