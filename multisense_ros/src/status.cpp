/**
 * @file status.cpp
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

#include <multisense_ros/status.h>

#include <multisense_ros/DeviceStatus.h>
#include <multisense_ros/PtpStatus.h>

namespace multisense_ros {

Status::Status(crl::multisense::Channel* driver):
    driver_(driver),
    device_nh_(""),
    status_pub_(),
    ptp_status_pub_(),
    status_timer_(),
    status_subscribers_(0),
    ptp_supported_(false),
    ptp_status_subscribers_(0)
{
    status_pub_ = device_nh_.advertise<multisense_ros::DeviceStatus>("status", 5,
                                                        std::bind(&Status::status_connect, this),
                                                        std::bind(&Status::status_disconnect, this));

    crl::multisense::system::DeviceInfo device_info;
    if (crl::multisense::Status_Ok == driver_->getDeviceInfo(device_info)) {
        ptp_supported_ = crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == device_info.hardwareRevision ||
                         crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == device_info.hardwareRevision ||
                         crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 == device_info.hardwareRevision ||
                         crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21i == device_info.hardwareRevision ||
                         crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO == device_info.hardwareRevision;
    }

    if (ptp_supported_) {
        ptp_status_pub_ = device_nh_.advertise<multisense_ros::PtpStatus>("ptp_status", 5,
                                                                          std::bind(&Status::ptp_status_connect, this),
                                                                          std::bind(&Status::ptp_status_disconnect, this));
    }

    status_timer_ = device_nh_.createTimer(ros::Duration(1), &Status::queryStatus, this);
}

Status::~Status()
{
}

void Status::queryStatus(const ros::TimerEvent& event)
{
    (void) event;

    if (status_subscribers_ > 0) {
        if (NULL != driver_) {
            crl::multisense::system::StatusMessage statusMessage;

            if (crl::multisense::Status_Ok == driver_->getDeviceStatus(statusMessage)) {
                multisense_ros::DeviceStatus deviceStatus;

                deviceStatus.time = ros::Time::now();
                deviceStatus.uptime = ros::Time(statusMessage.uptime);
                deviceStatus.systemOk = statusMessage.systemOk;
                deviceStatus.laserOk = statusMessage.laserOk;
                deviceStatus.laserMotorOk = statusMessage.laserMotorOk;
                deviceStatus.camerasOk = statusMessage.camerasOk;
                deviceStatus.imuOk = statusMessage.imuOk;
                deviceStatus.externalLedsOk = statusMessage.externalLedsOk;
                deviceStatus.processingPipelineOk = statusMessage.processingPipelineOk;
                deviceStatus.powerSupplyTemp = statusMessage.powerSupplyTemperature;
                deviceStatus.fpgaTemp = statusMessage.fpgaTemperature;
                deviceStatus.leftImagerTemp = statusMessage.leftImagerTemperature;
                deviceStatus.rightImagerTemp = statusMessage.rightImagerTemperature;
                deviceStatus.inputVoltage = statusMessage.inputVoltage;
                deviceStatus.inputCurrent = statusMessage.inputCurrent;
                deviceStatus.fpgaPower = statusMessage.fpgaPower;
                deviceStatus.logicPower = statusMessage.logicPower;
                deviceStatus.imagerPower = statusMessage.imagerPower;

                status_pub_.publish(deviceStatus);
            }

        }
    }

    if (ptp_supported_ && ptp_status_subscribers_ > 0)
    {
        if (NULL != driver_) {
            multisense_ros::PtpStatus ptpStatusMessage{};

            crl::multisense::system::PtpStatus ptpStatus;

            if (crl::multisense::Status_Ok == driver_->getPtpStatus(ptpStatus)) {
                ptpStatusMessage.valid = true;
                ptpStatusMessage.grandmaster_present = ptpStatus.gm_present != 0;

                if (ptpStatusMessage.grandmaster_present) {
                    memcpy(ptpStatusMessage.grandmaster_id.data(), ptpStatus.gm_id, 8);
                }
                ptpStatusMessage.grandmaster_offset = ptpStatus.gm_offset;
                ptpStatusMessage.path_delay = ptpStatus.path_delay;
                ptpStatusMessage.steps_removed = ptpStatus.steps_removed;
            }
            else
            {
                ptpStatusMessage.valid = false;
                ptpStatusMessage.grandmaster_present = false;
                ptpStatusMessage.grandmaster_offset = 0;
                ptpStatusMessage.path_delay = 0;
                ptpStatusMessage.steps_removed = 0;
            }

            ptp_status_pub_.publish(ptpStatusMessage);
        }
    }
}

void Status::status_connect()
{
    __sync_fetch_and_add(&status_subscribers_, 1);
}

void Status::status_disconnect()
{
    __sync_fetch_and_sub(&status_subscribers_, 1);
}

void Status::ptp_status_connect()
{
    __sync_fetch_and_add(&ptp_status_subscribers_, 1);
}

void Status::ptp_status_disconnect()
{
    __sync_fetch_and_sub(&ptp_status_subscribers_, 1);
}

}
