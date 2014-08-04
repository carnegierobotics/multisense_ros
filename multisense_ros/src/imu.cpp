/**
 * @file imu.cpp
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

#include <multisense_ros/imu.h>
#include <multisense_ros/RawImuData.h>
#include <std_msgs/Time.h>

using namespace crl::multisense;

namespace multisense_ros {

namespace { // anonymous

//
// Shim for C-style driver callbacks 

void imuCB(const imu::Header& header, void* userDataP)
{ reinterpret_cast<Imu*>(userDataP)->imuCallback(header); }


}; // anonymous

Imu::Imu(Channel* driver) :
    driver_(driver),
    device_nh_(""),
    imu_nh_(device_nh_, "imu"),
    accelerometer_pub_(),
    gyroscope_pub_(),
    magnetometer_pub_(),
    sub_lock_(),
    accel_subscribers_(0),
    gyro_subscribers_(0),
    mag_subscribers_(0),
    total_subscribers_(0)
{
    //
    // Get device info

    system::DeviceInfo  deviceInfo;
    Status status = driver_->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        ROS_ERROR("IMU: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    if (system::DeviceInfo::HARDWARE_REV_BCAM == deviceInfo.hardwareRevision) {
        ROS_INFO("hardware does not support an IMU");
        return;
    }

    system::VersionInfo v;
    status = driver_->getVersionInfo(v);
    if (Status_Ok != status) {
        ROS_ERROR("IMU: Unable to query sensor firmware version: %s",
                  Channel::statusString(status));
        return;
    }

    if (v.sensorFirmwareVersion < 0x0203)
        ROS_WARN("IMU support requires sensor firmware v2.3 or greater (sensor is running v%d.%d)",
                 v.sensorFirmwareVersion >> 8, v.sensorFirmwareVersion & 0xFF);
    else {

        //
        // Only publish IMU if we know firmware 2.3 or greater is running.

        driver_->stopStreams(Source_Imu);

        accelerometer_pub_ = imu_nh_.advertise<multisense_ros::RawImuData>("accelerometer", 20,
                                               boost::bind(&Imu::connect, this, imu::Sample::Type_Accelerometer),
                                               boost::bind(&Imu::disconnect, this, imu::Sample::Type_Accelerometer));
        gyroscope_pub_     = imu_nh_.advertise<multisense_ros::RawImuData>("gyroscope", 20,
                                               boost::bind(&Imu::connect, this, imu::Sample::Type_Gyroscope),
                                               boost::bind(&Imu::disconnect, this, imu::Sample::Type_Gyroscope));
        magnetometer_pub_  = imu_nh_.advertise<multisense_ros::RawImuData>("magnetometer", 20,
                                               boost::bind(&Imu::connect, this, imu::Sample::Type_Magnetometer),
                                               boost::bind(&Imu::disconnect, this, imu::Sample::Type_Magnetometer));
        driver_->addIsolatedCallback(imuCB, this);
    }
}

Imu::~Imu()
{
    driver_->stopStreams(Source_Imu);
    driver_->removeIsolatedCallback(imuCB);
}

void Imu::imuCallback(const imu::Header& header)
{    
    std::vector<imu::Sample>::const_iterator it = header.samples.begin();

    for(; it != header.samples.end(); ++it) {
        
        const imu::Sample& s = *it;

        multisense_ros::RawImuData msg;

        msg.time_stamp = ros::Time(s.timeSeconds,
                                   1000 * s.timeMicroSeconds);
        msg.x = s.x;
        msg.y = s.y;
        msg.z = s.z;

        switch(s.type) {
        case imu::Sample::Type_Accelerometer:

            if (accel_subscribers_ > 0)
                accelerometer_pub_.publish(msg);

            break;
        case imu::Sample::Type_Gyroscope:

            if (gyro_subscribers_ > 0)
                gyroscope_pub_.publish(msg);

            break;
        case imu::Sample::Type_Magnetometer:

            if (mag_subscribers_ > 0)
                magnetometer_pub_.publish(msg);

            break;
        }
    }        
}

void Imu::connect(imu::Sample::Type type)
{
    boost::mutex::scoped_lock lock(sub_lock_);

    switch(type) {
    case imu::Sample::Type_Accelerometer: accel_subscribers_++;   break;
    case imu::Sample::Type_Gyroscope:     gyro_subscribers_ ++;   break;
    case imu::Sample::Type_Magnetometer:  mag_subscribers_  ++;   break;
    default: ROS_WARN("unknown IMU subscription type: %d", type); return;
    }

    if (0 == total_subscribers_) {
        Status status = driver_->startStreams(Source_Imu);
        if (Status_Ok != status)
            ROS_ERROR("IMU: failed to start streams: %s",
                      Channel::statusString(status));
    }

    total_subscribers_ = accel_subscribers_ + gyro_subscribers_ + mag_subscribers_;
}

void Imu::disconnect(imu::Sample::Type type)
{
    boost::mutex::scoped_lock lock(sub_lock_);

    switch(type) {
    case imu::Sample::Type_Accelerometer: accel_subscribers_ --;  break;
    case imu::Sample::Type_Gyroscope:     gyro_subscribers_  --;  break;
    case imu::Sample::Type_Magnetometer:  mag_subscribers_   --;  break;
    default: ROS_WARN("unknown IMU subscription type: %d", type); return;
    }

    total_subscribers_ = accel_subscribers_ + gyro_subscribers_ + mag_subscribers_;

    if (total_subscribers_ <= 0){
        Status status = driver_->stopStreams(Source_Imu);
        if (Status_Ok != status)
            ROS_ERROR("IMU: failed to stop streams: %s",
                      Channel::statusString(status));
    }
}

} // namespace
