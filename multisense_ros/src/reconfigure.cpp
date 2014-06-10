/**
 * @file reconfigure.cpp
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

#include <multisense_ros/reconfigure.h>

using namespace crl::multisense;

namespace multisense_ros {

Reconfigure::Reconfigure(Channel* driver,
                         boost::function<void ()> resolutionChangeCallback) :
    driver_(driver),
    resolution_change_callback_(resolutionChangeCallback),
    device_nh_(""),
    device_modes_(),
    imu_samples_per_message_(0),
    imu_configs_(),
    server_sl_bm_cmv2000_(),
    server_sl_bm_cmv2000_imu_(),
    server_sl_bm_cmv4000_(),
    server_sl_bm_cmv4000_imu_(),
    server_sl_sgm_cmv2000_imu_(),
    server_sl_sgm_cmv4000_imu_(),
    server_bcam_imx104_(),
    lighting_supported_(true),
    motor_supported_(true)
{
    system::DeviceInfo  deviceInfo;
    system::VersionInfo versionInfo;

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

    //
    // Launch the correct reconfigure server for this device configuration.

    if (system::DeviceInfo::HARDWARE_REV_BCAM == deviceInfo.hardwareRevision) {

        server_bcam_imx104_ =
            boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config> > (
                new dynamic_reconfigure::Server<multisense_ros::bcam_imx104Config>(device_nh_));
        server_bcam_imx104_->setCallback(boost::bind(&Reconfigure::callback_bcam_imx104, this, _1, _2));

    } else if (versionInfo.sensorFirmwareVersion <= 0x0202) {

        switch(deviceInfo.imagerType) {
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:

            server_sl_bm_cmv2000_ =
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000Config>(device_nh_));
            server_sl_bm_cmv2000_->setCallback(boost::bind(&Reconfigure::callback_sl_bm_cmv2000, this, _1, _2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_bm_cmv4000_ =
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000Config>(device_nh_));
            server_sl_bm_cmv4000_->setCallback(boost::bind(&Reconfigure::callback_sl_bm_cmv4000, this, _1, _2));

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
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv2000_imuConfig>(device_nh_));
            server_sl_bm_cmv2000_imu_->setCallback(boost::bind(&Reconfigure::callback_sl_bm_cmv2000_imu, this, _1, _2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_bm_cmv4000_imu_ =
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_bm_cmv4000_imuConfig>(device_nh_));
            server_sl_bm_cmv4000_imu_->setCallback(boost::bind(&Reconfigure::callback_sl_bm_cmv4000_imu, this, _1, _2));

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
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv2000_imuConfig>(device_nh_));
            server_sl_sgm_cmv2000_imu_->setCallback(boost::bind(&Reconfigure::callback_sl_sgm_cmv2000_imu, this, _1, _2));

            break;
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
        case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:

            server_sl_sgm_cmv4000_imu_ =
                boost::shared_ptr< dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig> > (
                    new dynamic_reconfigure::Server<multisense_ros::sl_sgm_cmv4000_imuConfig>(device_nh_));
            server_sl_sgm_cmv4000_imu_->setCallback(boost::bind(&Reconfigure::callback_sl_sgm_cmv4000_imu, this, _1, _2));

            break;
        default:
            
            ROS_ERROR("Reconfigure: unsupported imager type \"%d\"", deviceInfo.imagerType);
            return;
        }
    }
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

    //
    // Set all other image config from dynamic reconfigure

    cfg.setFps(dyn.fps);
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

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {

        if (false == resolution_change_callback_.empty())
            resolution_change_callback_();

        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }

    //
    // Send the desired motor speed

    if (motor_supported_) {

        const float radiansPerSecondToRpm = 9.54929659643;

        status = driver_->setMotorSpeed(radiansPerSecondToRpm * dyn.motor_speed);
        if (Status_Ok != status) {
            if (Status_Unsupported == status)
                motor_supported_ = false;
            else
                ROS_ERROR("Reconfigure: failed to set motor speed: %s", 
                          Channel::statusString(status));
        }
    }

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

        status = driver_->setLightingConfig(leds);
        if (Status_Ok != status) {
            if (Status_Unsupported == status)
                lighting_supported_ = false;
            else
                ROS_ERROR("Reconfigure: failed to set lighting config: %s", 
                          Channel::statusString(status));
        }
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
        configureCamera(cfg, dyn);                              \
    } while(0)

#define SL_BM_IMU()  do {                                       \
        GET_CONFIG();                                           \
        configureCamera(cfg, dyn);                              \
        configureImu(dyn);                                      \
    } while(0)

#define SL_SGM_IMU()  do {                                      \
        GET_CONFIG();                                           \
        configureSgm(cfg, dyn);                                 \
        configureCamera(cfg, dyn);                              \
        configureImu(dyn);                                      \
    } while(0)

//
// The dynamic reconfigure callbacks (MultiSense S* variations)

void Reconfigure::callback_sl_bm_cmv2000      (multisense_ros::sl_bm_cmv2000Config&      dyn, uint32_t level) { SL_BM();      };
void Reconfigure::callback_sl_bm_cmv2000_imu  (multisense_ros::sl_bm_cmv2000_imuConfig&  dyn, uint32_t level) { SL_BM_IMU();  };
void Reconfigure::callback_sl_bm_cmv4000      (multisense_ros::sl_bm_cmv4000Config&      dyn, uint32_t level) { SL_BM();      };
void Reconfigure::callback_sl_bm_cmv4000_imu  (multisense_ros::sl_bm_cmv4000_imuConfig&  dyn, uint32_t level) { SL_BM_IMU();  };
void Reconfigure::callback_sl_sgm_cmv2000_imu (multisense_ros::sl_sgm_cmv2000_imuConfig& dyn, uint32_t level) { SL_SGM_IMU(); };
void Reconfigure::callback_sl_sgm_cmv4000_imu (multisense_ros::sl_sgm_cmv4000_imuConfig& dyn, uint32_t level) { SL_SGM_IMU(); };

//
// BCAM (Sony IMX104)

void Reconfigure::callback_bcam_imx104(multisense_ros::bcam_imx104Config& dyn, 
                                       uint32_t                           level) 
{
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

    //
    // If we are changing the resolution, let others know about it

    if (resolutionChange) {

        if (false == resolution_change_callback_.empty())
            resolution_change_callback_();

        status = driver_->startStreams(streamsEnabled);
        if (Status_Ok != status)
            ROS_ERROR("Reconfigure: failed to restart streams after a resolution change: %s",
                      Channel::statusString(status));
    }
}

} // namespace
