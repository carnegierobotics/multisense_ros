/**
 * @file reconfigure.h
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#ifndef MULTISENSE_ROS_RECONFIGURE_H
#define MULTISENSE_ROS_RECONFIGURE_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <multisense_lib/MultiSenseChannel.hh>

#include <dynamic_reconfigure/server.h>
#include <multisense_ros/sl_bm_cmv2000Config.h>
#include <multisense_ros/sl_bm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_bm_cmv4000Config.h>
#include <multisense_ros/sl_bm_cmv4000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv2000_imuConfig.h>
#include <multisense_ros/sl_sgm_cmv4000_imuConfig.h>
#include <multisense_ros/bcam_imx104Config.h>

namespace multisense_ros {

class Reconfigure {
public:

    Reconfigure(crl::multisense::Channel* driver,
                boost::function<void ()> resolutionChangeCallback=0);
                
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

    //
    // Internal helper functions

    bool changeResolution(crl::multisense::image::Config& cfg,
                          int32_t width, int32_t height, int32_t disparities);
    template<class T> void configureSgm(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureCamera(crl::multisense::image::Config& cfg, const T& dyn);
    template<class T> void configureImu(const T& dyn);

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Resolution change callback

    boost::function<void ()> resolution_change_callback_;

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

    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config> >      server_sl_bm_cmv2000_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig> >  server_sl_bm_cmv2000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config> >      server_sl_bm_cmv4000_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig> >  server_sl_bm_cmv4000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig> > server_sl_sgm_cmv2000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig> > server_sl_sgm_cmv4000_imu_;
    boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config> >        server_bcam_imx104_;

    //
    // Cached values for supported sub-systems (these may be unavailable on
    // certain hardware variations: S7, S7S, M, etc.)

    bool lighting_supported_;
    bool motor_supported_;
};

} // multisense_ros

#endif
