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

Imu::Imu(Channel* driver, std::string tf_prefix) :
    driver_(driver),
    device_nh_(""),
    imu_nh_(device_nh_, "imu"),
    accelerometer_pub_(),
    gyroscope_pub_(),
    magnetometer_pub_(),
    imu_pub_(),
    accelerometer_vector_pub_(),
    gyroscope_vector_pub_(),
    magnetometer_vector_pub_(),
    imu_message_(),
    sub_lock_(),
    total_subscribers_(0),
    tf_prefix_(tf_prefix),
    accel_frameId_(tf_prefix_ + "/accel"),
    gyro_frameId_(tf_prefix_ + "/gyro"),
    mag_frameId_(tf_prefix_ + "/mag")
{

    //
    // Initialize the sensor_msgs::Imu topic
    // We will publish the data in the accelerometer frame applying the
    // transform from the /gyro to the /accel frame to the gyroscope data

    imu_message_.header.frame_id = accel_frameId_;

    //
    // Covariance matrix for linear acceleration and angular velocity were
    // generated using 2 minutes of logged data with the default imu
    // settings. Note the angular velocity covariance has the nominal gyro
    // to accelerometer transform applied to it

    imu_message_.linear_acceleration_covariance[0] = 6.98179077e-04;
    imu_message_.linear_acceleration_covariance[1] = 2.46789341e-06;
    imu_message_.linear_acceleration_covariance[2] =-2.50549745e-06;
    imu_message_.linear_acceleration_covariance[3] = 2.46789341e-06;
    imu_message_.linear_acceleration_covariance[4] = 5.02177646e-04;
    imu_message_.linear_acceleration_covariance[5] = 5.26265558e-05;
    imu_message_.linear_acceleration_covariance[6] =-2.50549745e-06;
    imu_message_.linear_acceleration_covariance[7] = 5.26265558e-05;
    imu_message_.linear_acceleration_covariance[8] = 9.22796937e-04;

    imu_message_.angular_velocity_covariance[0] = 8.79376936e-06;
    imu_message_.angular_velocity_covariance[1] = -3.56007627e-07;
    imu_message_.angular_velocity_covariance[2] = 2.22611968e-07;
    imu_message_.angular_velocity_covariance[3] = -3.56007627e-07;
    imu_message_.angular_velocity_covariance[4] = 8.78939245e-06;
    imu_message_.angular_velocity_covariance[5] = -8.08367486e-07;
    imu_message_.angular_velocity_covariance[6] = 1.33981577e-05;
    imu_message_.angular_velocity_covariance[7] = 2.22611968e-07;
    imu_message_.angular_velocity_covariance[8] = -8.08367486e-07;

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
                                               boost::bind(&Imu::startStreams, this),
                                               boost::bind(&Imu::stopStreams, this));
        gyroscope_pub_     = imu_nh_.advertise<multisense_ros::RawImuData>("gyroscope", 20,
                                               boost::bind(&Imu::startStreams, this),
                                               boost::bind(&Imu::stopStreams, this));
        magnetometer_pub_  = imu_nh_.advertise<multisense_ros::RawImuData>("magnetometer", 20,
                                               boost::bind(&Imu::startStreams, this),
                                               boost::bind(&Imu::stopStreams, this));
        imu_pub_           = imu_nh_.advertise<sensor_msgs::Imu>("imu_data", 20,
                                               boost::bind(&Imu::startStreams, this),
                                               boost::bind(&Imu::stopStreams, this));

        accelerometer_vector_pub_ = imu_nh_.advertise<geometry_msgs::Vector3Stamped>("accelerometer_vector", 20,
                                                      boost::bind(&Imu::startStreams, this),
                                                      boost::bind(&Imu::stopStreams, this));
        gyroscope_vector_pub_     = imu_nh_.advertise<geometry_msgs::Vector3Stamped>("gyroscope_vector", 20,
                                                      boost::bind(&Imu::startStreams, this),
                                                      boost::bind(&Imu::stopStreams, this));
        magnetometer_vector_pub_  = imu_nh_.advertise<geometry_msgs::Vector3Stamped>("magnetometer_vector", 20,
                                                      boost::bind(&Imu::startStreams, this),
                                                      boost::bind(&Imu::stopStreams, this));

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

    uint32_t accel_subscribers = accelerometer_pub_.getNumSubscribers();
    uint32_t gyro_subscribers = gyroscope_pub_.getNumSubscribers();
    uint32_t mag_subscribers = magnetometer_pub_.getNumSubscribers();
    uint32_t imu_subscribers = imu_pub_.getNumSubscribers();
    uint32_t accel_vector_subscribers = accelerometer_vector_pub_.getNumSubscribers();
    uint32_t gyro_vector_subscribers = gyroscope_vector_pub_.getNumSubscribers();
    uint32_t mag_vector_subscribers = magnetometer_vector_pub_.getNumSubscribers();

    for(; it != header.samples.end(); ++it) {

        const imu::Sample& s = *it;

        multisense_ros::RawImuData msg;
        geometry_msgs::Vector3Stamped vector_msg;

        msg.time_stamp = ros::Time(s.timeSeconds,
                                   1000 * s.timeMicroSeconds);
        msg.x = s.x;
        msg.y = s.y;
        msg.z = s.z;

        vector_msg.header.stamp = msg.time_stamp;
        vector_msg.vector.x = s.x;
        vector_msg.vector.y = s.y;
        vector_msg.vector.z = s.z;

        imu_message_.header.stamp = msg.time_stamp;

        switch(s.type) {
        case imu::Sample::Type_Accelerometer:
            //
            // Convert from g to m/s^2

            imu_message_.linear_acceleration.x = s.x * 9.80665;
            imu_message_.linear_acceleration.y = s.y * 9.80665;
            imu_message_.linear_acceleration.z = s.z * 9.80665;


            if (accel_subscribers > 0)
                accelerometer_pub_.publish(msg);

            if (imu_subscribers > 0)
                imu_pub_.publish(imu_message_);

            if (accel_vector_subscribers > 0) {
                vector_msg.header.frame_id = accel_frameId_;
                accelerometer_vector_pub_.publish(vector_msg);
            }

            break;
        case imu::Sample::Type_Gyroscope:

            //
            // Convert from deg/sec to rad/sec and apply the nominal
            // calibration from the gyro to the accelerometer. Since all points
            // on a rigid body have the same angular velocity only the rotation
            // about the z axis of 90 degrees needs to be applied. (i.e.
            // new_x = orig_y ; new_y = -orig_x)

            imu_message_.angular_velocity.x = s.y * M_PI/180.;
            imu_message_.angular_velocity.y = -s.x * M_PI/180.;
            imu_message_.angular_velocity.z = s.z * M_PI/180.;


            if (gyro_subscribers > 0)
                gyroscope_pub_.publish(msg);

            if (imu_subscribers > 0)
                imu_pub_.publish(imu_message_);

            if (gyro_vector_subscribers > 0) {
                vector_msg.header.frame_id = gyro_frameId_;
                gyroscope_vector_pub_.publish(vector_msg);
            }

            break;
        case imu::Sample::Type_Magnetometer:


            if (mag_subscribers > 0)
                magnetometer_pub_.publish(msg);

            if (mag_vector_subscribers > 0) {
                vector_msg.header.frame_id = mag_frameId_;
                magnetometer_vector_pub_.publish(vector_msg);
            }

            break;
        }
    }
}


void Imu::startStreams()
{
    if (0 == total_subscribers_) {
        Status status = driver_->startStreams(Source_Imu);
        if (Status_Ok != status)
            ROS_ERROR("IMU: failed to start streams: %s",
                      Channel::statusString(status));
    }

    total_subscribers_ = accelerometer_pub_.getNumSubscribers()
                       + gyroscope_pub_.getNumSubscribers()
                       + magnetometer_pub_.getNumSubscribers()
                       + imu_pub_.getNumSubscribers();
}

void Imu::stopStreams()
{
    total_subscribers_ = accelerometer_pub_.getNumSubscribers()
                       + gyroscope_pub_.getNumSubscribers()
                       + magnetometer_pub_.getNumSubscribers()
                       + imu_pub_.getNumSubscribers();

    if (total_subscribers_ <= 0){
        Status status = driver_->stopStreams(Source_Imu);
        if (Status_Ok != status)
            ROS_ERROR("IMU: failed to stop streams: %s",
                      Channel::statusString(status));
    }
}

} // namespace
