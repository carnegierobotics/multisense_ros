/**
 * @file pps.cpp
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

#include <multisense_ros/pps.h>
#include <std_msgs/Time.h>

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

    pps_msg.data = ros::Time(header.sensorTime / 1000000000ll,
                             header.sensorTime % 1000000000ll);

    pps_pub_.publish(pps_msg);
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
