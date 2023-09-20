/**
 * @file statistics.cpp
 *
 * Copyright 2023
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

#include <multisense_ros/statistics.h>

#include <multisense_ros/ChannelStatistics.h>

namespace multisense_ros {

Statistics::Statistics(crl::multisense::Channel* driver):
    driver_(driver),
    device_nh_(""),
    statistics_pub_(),
    statistics_timer_(),
    subscribers_(0)
{
    statistics_pub_ = device_nh_.advertise<multisense_ros::ChannelStatistics>("channel_statistics", 5,
                                                        std::bind(&Statistics::connect, this),
                                                        std::bind(&Statistics::disconnect, this));

    statistics_timer_ = device_nh_.createTimer(ros::Duration(1), &Statistics::queryStatistics, this);
}

Statistics::~Statistics()
{
}

void Statistics::queryStatistics(const ros::TimerEvent& event)
{
    (void) event;

    if (subscribers_ <= 0)
        return;

    if (NULL != driver_)
    {
        crl::multisense::system::ChannelStatistics statistics = driver_->getStats();

        multisense_ros::ChannelStatistics statisticsMsg;

        statisticsMsg.header.stamp = ros::Time::now();

        statisticsMsg.num_missed_headers = statistics.numMissedHeaders;
        statisticsMsg.num_dropped_assemblers = statistics.numDroppedAssemblers;
        statisticsMsg.num_image_meta_data = statistics.numImageMetaData;
        statisticsMsg.num_dispatched_image = statistics.numDispatchedImage;
        statisticsMsg.num_dispatched_lidar = statistics.numDispatchedLidar;
        statisticsMsg.num_dispatched_pps = statistics.numDispatchedPps;
        statisticsMsg.num_dispatched_imu = statistics.numDispatchedImu;
        statisticsMsg.num_dispatched_compressed_image = statistics.numDispatchedCompressedImage;
        statisticsMsg.num_dispatched_ground_surface_spline = statistics.numDispatchedGroundSurfaceSpline;
        statisticsMsg.num_dispatched_april_tag_detections = statistics.numDispatchedAprilTagDetections;

        statistics_pub_.publish(statisticsMsg);
    }
}

void Statistics::connect()
{
    __sync_fetch_and_add(&subscribers_, 1);
}

void Statistics::disconnect()
{
    __sync_fetch_and_sub(&subscribers_, 1);
}


}
