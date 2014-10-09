/**
 * @file pps.cpp
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

#include <multisense_ros/pps.h>
#include <std_msgs/Time.h>

#include <multisense_ros/StampedPps.h>

using namespace crl::multisense;

namespace multisense_ros {

namespace { // anonymous

//
// Shim for C-style driver callbacks

void ppsCB(const pps::Header& header, void* userDataP)
{ reinterpret_cast<Pps*>(userDataP)->ppsCallback(header); }


}; // anonymous

Pps::Pps(Channel* driver) :
    driver_(driver),
    device_nh_(""),
    pps_pub_(),
    stamped_pps_pub_(),
    subscribers_(0)
{
    system::DeviceInfo deviceInfo;
    Status status = driver_->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        ROS_ERROR("Camera: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    if (system::DeviceInfo::HARDWARE_REV_BCAM == deviceInfo.hardwareRevision) {
        ROS_INFO("hardware does not support PPS");
        return;
    }

    system::VersionInfo v;
    if (Status_Ok == driver_->getVersionInfo(v) && v.sensorFirmwareVersion < 0x0202)
        ROS_ERROR("PPS support requires sensor firmware v2.2 or greater (sensor is running v%d.%d)\n",
                  v.sensorFirmwareVersion >> 8, v.sensorFirmwareVersion & 0xFF);
    else {

        //
        // Only publish PPS if we know firmware 2.2 or greater is running.
        //
        // 2.1 firmware had a bug where PPS events could (rarely) be published with
        // the previous event's timecode.

        pps_pub_ = device_nh_.advertise<std_msgs::Time>("pps", 5,
                                                        boost::bind(&Pps::connect, this),
                                                        boost::bind(&Pps::disconnect, this));

        stamped_pps_pub_ = device_nh_.advertise<multisense_ros::StampedPps>("stamped_pps", 5,
                                                        boost::bind(&Pps::connect, this),
                                                        boost::bind(&Pps::disconnect, this));
        driver_->addIsolatedCallback(ppsCB, this);
    }
}

Pps::~Pps()
{
    driver_->removeIsolatedCallback(ppsCB);
}

void Pps::ppsCallback(const pps::Header& header)
{
    if (subscribers_ <= 0)
        return;

    std_msgs::Time pps_msg;

    multisense_ros::StampedPps stamped_pps_msg;

    pps_msg.data = ros::Time(header.sensorTime / 1000000000ll,
                             header.sensorTime % 1000000000ll);


    stamped_pps_msg.data = pps_msg.data;

    stamped_pps_msg.host_time = ros::Time(header.timeSeconds,
                                          1000 * header.timeMicroSeconds);

    pps_pub_.publish(pps_msg);

    stamped_pps_pub_.publish(stamped_pps_msg);
}

void Pps::connect()
{
    __sync_fetch_and_add(&subscribers_, 1);
}

void Pps::disconnect()
{
    __sync_fetch_and_sub(&subscribers_, 1);
}

} // namespace
