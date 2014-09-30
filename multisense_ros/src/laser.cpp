/**
 * @file laser.cpp
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

#include <multisense_ros/laser.h>
#include <multisense_ros/RawLidarData.h>
#include <multisense_ros/RawLidarCal.h>
#include <angles/angles.h>

#include <arpa/inet.h>

using namespace crl::multisense;

namespace { // anonymous

const uint32_t laser_cloud_step = 16;

tf::Transform makeTransform(float T[4][4])
{
    //
    // Manually create the rotation matrix
    tf::Matrix3x3 rot = tf::Matrix3x3(T[0][0],
                                      T[0][1],
                                      T[0][2],
                                      T[1][0],
                                      T[1][1],
                                      T[1][2],
                                      T[2][0],
                                      T[2][1],
                                      T[2][2]);

    //
    // Maually create the translation vector
    tf::Vector3 trans = tf::Vector3(T[0][3], T[1][3], T[2][3]);

    return tf::Transform(rot, trans);
}



}; // anonymous

namespace multisense_ros {

const float Laser::EXPECTED_RATE = 40.0;

namespace { // anonymous

//
// Shims for c-style driver callbacks

void lCB(const lidar::Header&        header,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->scanCallback(header);
}

void pCB(const lidar::Header&        header,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->pointCloudCallback(header);
}

}; // anonymous

Laser::Laser(Channel* driver,
             const std::string& tf_prefix,
             const std::string& robot_desc) :
    driver_(driver),
    subscribers_(0),
    spindle_angle_(0.0),
    previous_scan_time_(0.0)
{

    //
    // Get device info

    system::DeviceInfo  deviceInfo;

    Status status = driver_->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        ROS_ERROR("Laser: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    switch(deviceInfo.hardwareRevision) {
    case system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL:

        ; // ok, this one has a laser

        break;
    default:

        ROS_INFO("hardware does not support a laser");
        return;
    }

    ros::NodeHandle nh("");

    //
    // Set frame ID

    frame_id_ = tf_prefix + "/head_hokuyo_frame";

    left_camera_optical_ =  tf_prefix + "/" + "left_camera_optical_frame";
    motor_               =  tf_prefix + "/" + "motor";
    spindle_             =  tf_prefix + "/" + "spindle";
    hokuyo_              =  tf_prefix + "/" + "hokuyo_link";

    ROS_INFO("laser frame id: %s", frame_id_.c_str());

    //
    // Stop lidar stream

    stop();

    //
    // Query calibration from sensor

    status = driver_->getLidarCalibration(lidar_cal_);
    if (Status_Ok != status)
        ROS_WARN("could not query lidar calibration (%s), using URDF defaults",
                 Channel::statusString(status));
    else {

        //
        // Create two static transforms representing sensor calibration
        motor_to_camera_ = makeTransform(lidar_cal_.cameraToSpindleFixed);
        laser_to_spindle_ = makeTransform(lidar_cal_.laserToSpindle);

    }

    //
    // Create scan publisher

    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("lidar_scan", 20,
                boost::bind(&Laser::subscribe, this),
                boost::bind(&Laser::unsubscribe, this));

    //
    // Initialize point cloud structure

    point_cloud_.is_bigendian    = (htonl(1) == 1);
    point_cloud_.is_dense        = true;
    point_cloud_.point_step      = laser_cloud_step;
    point_cloud_.height          = 1;
    point_cloud_.header.frame_id =  tf_prefix + "/left_camera_optical_frame";

    point_cloud_.fields.resize(4);
    point_cloud_.fields[0].name     = "x";
    point_cloud_.fields[0].offset   = 0;
    point_cloud_.fields[0].count    = 1;
    point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    point_cloud_.fields[1].name     = "y";
    point_cloud_.fields[1].offset   = 4;
    point_cloud_.fields[1].count    = 1;
    point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    point_cloud_.fields[2].name     = "z";
    point_cloud_.fields[2].offset   = 8;
    point_cloud_.fields[2].count    = 1;
    point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    point_cloud_.fields[3].name     = "intensity";
    point_cloud_.fields[3].offset   = 12;
    point_cloud_.fields[3].count    = 1;
    point_cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    //
    // Create point cloud publisher

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_points2", 5,
                       boost::bind(&Laser::subscribe, this),
                       boost::bind(&Laser::unsubscribe, this));

    //
    // Create calibration publishers

    ros::NodeHandle calibration_nh(nh, "calibration");
    raw_lidar_cal_pub_  = calibration_nh.advertise<multisense_ros::RawLidarCal>("raw_lidar_cal", 1, true);
    raw_lidar_data_pub_ = calibration_nh.advertise<multisense_ros::RawLidarData>("raw_lidar_data", 20,
                          boost::bind(&Laser::subscribe, this),
                          boost::bind(&Laser::unsubscribe, this));

    //
    // Publish calibration

    multisense_ros::RawLidarCal ros_msg;

    const float *calP = reinterpret_cast<const float*>(&(lidar_cal_.laserToSpindle[0][0]));
    for(uint32_t i=0; i<16; ++i)
        ros_msg.laserToSpindle[i] = calP[i];

    calP = reinterpret_cast<const float*>(&(lidar_cal_.cameraToSpindleFixed[0][0]));
    for(uint32_t i=0; i<16; ++i)
        ros_msg.cameraToSpindleFixed[i] = calP[i];

    raw_lidar_cal_pub_.publish(ros_msg);


    //
    // Populate the jointstates message for publishing the laser spindle
    // angle

    joint_states_.name.resize(1);
    joint_states_.position.resize(1);
    joint_states_.velocity.resize(1);
    joint_states_.name[0] = "motor_joint";
    joint_states_.position[0] = 0.0;
    joint_states_.velocity[0] = 0.0;

    //
    // Create a joint state publisher

    joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);

    //
    // Create a timer routine to publish the laser transform even when nothing
    // is subscribed to the laser topics. Publishing occurs at 1Hz

    timer_ = nh.createTimer(ros::Duration(1), &Laser::defaultTfPublisher, this);

    //
    // Register callbacks, driver creates dedicated background thread for each

    driver_->addIsolatedCallback(lCB, this);
    driver_->addIsolatedCallback(pCB, this);


}

Laser::~Laser()
{
    boost::mutex::scoped_lock lock(sub_lock_);
    stop();
    driver_->removeIsolatedCallback(lCB);
    driver_->removeIsolatedCallback(pCB);
}


void Laser::pointCloudCallback(const lidar::Header& header)
{
    //
    // Get out if we have no work to do

    if (0 == point_cloud_pub_.getNumSubscribers())
        return;

    point_cloud_.data.resize(laser_cloud_step * header.pointCount);
    point_cloud_.row_step     = header.pointCount;
    point_cloud_.width        = header.pointCount;
    point_cloud_.header.stamp = ros::Time(header.timeStartSeconds,
                                          1000 * header.timeStartMicroSeconds);
    //
    // For convenience below

    uint8_t       *cloudP            = reinterpret_cast<uint8_t*>(&point_cloud_.data[0]);
    const uint32_t pointSize         = 3 * sizeof(float); // x, y, z
    const double   arcRadians        = 1e-6 * static_cast<double>(header.scanArc);
    const double   mirrorThetaStart  = -arcRadians / 2.0;
    const double   spindleAngleStart = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleStart));
    const double   spindleAngleEnd   = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleEnd));
    const double   spindleAngleRange = angles::normalize_angle(spindleAngleEnd - spindleAngleStart);

    for(uint32_t i=0; i<header.pointCount; ++i, cloudP += laser_cloud_step) {

        //
        // Percent through the scan arc

        const double percent = static_cast<double>(i) / static_cast<double>(header.pointCount - 1);

        //
        // The mirror angle for this point

        const double mirrorTheta = mirrorThetaStart + percent * arcRadians;

        //
        // The rotation about the spindle

        const double spindleTheta = spindleAngleStart + percent * spindleAngleRange;

        tf::Transform spindle_to_motor = getSpindleTransform(spindleTheta);

        //
        // The coordinate in left optical frame

        const double      rangeMeters = 1e-3 * static_cast<double>(header.rangesP[i]);  // from millimeters
        const tf::Vector3 pointMotor  = (laser_to_spindle_ *
                                         tf::Vector3(rangeMeters * std::sin(mirrorTheta), 0.0,
                                                     rangeMeters *  std::cos(mirrorTheta)));
        const tf::Vector3 pointCamera = motor_to_camera_ * (spindle_to_motor * pointMotor);

        //
        // Copy data to point cloud structure

        const float xyz[3] = {static_cast<float>(pointCamera.getX()),
                              static_cast<float>(pointCamera.getY()),
                              static_cast<float>(pointCamera.getZ())};

        memcpy(cloudP, &(xyz[0]), pointSize);
        float* intensityChannel = reinterpret_cast<float*>(cloudP + pointSize);
        *intensityChannel = static_cast<float>(header.intensitiesP[i]);   // in device units
    }

    point_cloud_pub_.publish(point_cloud_);
}

void Laser::scanCallback(const lidar::Header& header)
{

    const ros::Time start_absolute_time = ros::Time(header.timeStartSeconds,
                                                    1000 * header.timeStartMicroSeconds);
    const ros::Time end_absolute_time   = ros::Time(header.timeEndSeconds,
                                                    1000 * header.timeEndMicroSeconds);
    const ros::Time scan_time((end_absolute_time - start_absolute_time).toSec());

    const float angle_start = 1e-6 * static_cast<float>(header.spindleAngleStart);
    const float angle_end   = 1e-6 * static_cast<float>(header.spindleAngleEnd);

    publishStaticTransforms(start_absolute_time);

    //
    // Initialize the previous scan time to the start time if it has not
    // been previously set

    if (previous_scan_time_.is_zero())
    {
        previous_scan_time_ = start_absolute_time;
    }

    //
    // Check that our laser data is being published at the EXPECTED_RATE within
    // a 10% margin

    const float laser_period = scan_time.toSec() + (start_absolute_time - previous_scan_time_).toSec();
    if ( 1.0/laser_period < (EXPECTED_RATE - EXPECTED_RATE * 0.1)) {
        ROS_WARN("The laser scan frequency %f is less than the expected frequency of %f",
                1.0/laser_period, EXPECTED_RATE);
    }

    //
    // Compute the velocity between our last scan and the start of our current
    // scan

    float velocity = angles::normalize_angle((angle_start - spindle_angle_)) /
        (start_absolute_time - previous_scan_time_).toSec();

    publishSpindleTransform(angle_start, velocity, start_absolute_time);
    spindle_angle_ = angle_start;

    //
    // Compute the velocity for the spindle during the duration of our
    // laser scan

    velocity = angles::normalize_angle((angle_end - angle_start)) / scan_time.toSec();

    publishSpindleTransform(angle_end, velocity, end_absolute_time);
    spindle_angle_ = angle_end;
    previous_scan_time_ = end_absolute_time;

    if (scan_pub_.getNumSubscribers() > 0) {

        const double arcRadians = 1e-6 * static_cast<double>(header.scanArc);

        laser_msg_.header.frame_id = frame_id_;
        laser_msg_.header.stamp    = start_absolute_time;
        laser_msg_.scan_time       = scan_time.toSec();
        laser_msg_.time_increment  = laser_msg_.scan_time / header.pointCount;
        laser_msg_.angle_min       = -arcRadians / 2.0;
        laser_msg_.angle_max       = arcRadians / 2.0;
        laser_msg_.angle_increment = arcRadians / (header.pointCount - 1);
        laser_msg_.range_min       = 0.0;
        laser_msg_.range_max       = static_cast<double>(header.maxRange) / 1000.0;

        laser_msg_.ranges.resize(header.pointCount);
        laser_msg_.intensities.resize(header.pointCount);

        for (size_t i=0; i<header.pointCount; i++) {
            laser_msg_.ranges[i]      = 1e-3 * static_cast<float>(header.rangesP[i]); // from millimeters
            laser_msg_.intensities[i] = static_cast<float>(header.intensitiesP[i]);   // in device units
        }

        scan_pub_.publish(laser_msg_);
    }

    if (raw_lidar_data_pub_.getNumSubscribers() > 0) {

        RawLidarData::Ptr ros_msg(new RawLidarData);

        ros_msg->scan_count  = header.scanId;
        ros_msg->time_start  = start_absolute_time;
        ros_msg->time_end    = end_absolute_time;
        ros_msg->angle_start = header.spindleAngleStart;
        ros_msg->angle_end   = header.spindleAngleEnd;

        ros_msg->distance.resize(header.pointCount);
        memcpy(&(ros_msg->distance[0]),
               header.rangesP,
               header.pointCount * sizeof(uint32_t));

        ros_msg->intensity.resize(header.pointCount);
        memcpy(&(ros_msg->intensity[0]),
               header.intensitiesP,
               header.pointCount * sizeof(uint32_t));

        raw_lidar_data_pub_.publish(ros_msg);
    }
}

void Laser::publishStaticTransforms(const ros::Time& time) {

    //
    // Publish the static transforms from our calibration.
    static_tf_broadcaster_.sendTransform(tf::StampedTransform(motor_to_camera_,
                                          time,left_camera_optical_,
                                          motor_));



    static_tf_broadcaster_.sendTransform(tf::StampedTransform(laser_to_spindle_,
                                          time, spindle_, hokuyo_));


}

void Laser::publishSpindleTransform(const float spindle_angle, const float velocity, const ros::Time& time) {
    joint_states_.header.stamp = time;
    joint_states_.position[0] = spindle_angle;
    joint_states_.velocity[0] = velocity;
    joint_states_pub_.publish(joint_states_);
}

tf::Transform Laser::getSpindleTransform(float spindle_angle){

    //
    // Spindle angle turns about the z-axis to create a transform where it adjusts
    // yaw
    tf::Matrix3x3 spindle_rot = tf::Matrix3x3();
    spindle_rot.setRPY(0.0, 0.0, spindle_angle);
    tf::Transform spindle_to_motor = tf::Transform(spindle_rot);

    return spindle_to_motor;
}

void Laser::defaultTfPublisher(const ros::TimerEvent& event){
    //
    // If our message time is 0 or our message time is over 1 second old
    // we are not subscribed to a laser topic anymore. Publish the default
    // transform
    if ( (laser_msg_.header.stamp.is_zero() ||
         (ros::Time::now() - laser_msg_.header.stamp >= ros::Duration(1))) &&
         (point_cloud_.header.stamp.is_zero() ||
         (ros::Time::now() - point_cloud_.header.stamp >= ros::Duration(1))) )

    {
        publishStaticTransforms(ros::Time::now());
        publishSpindleTransform(spindle_angle_, 0.0, ros::Time::now());
    }
}

void Laser::stop()
{
    subscribers_ = 0;

    Status status = driver_->stopStreams(Source_Lidar_Scan);
    if (Status_Ok != status)
        ROS_ERROR("Laser: failed to stop laser stream: %s",
                  Channel::statusString(status));
}

void Laser::unsubscribe()
{
    boost::mutex::scoped_lock lock(sub_lock_);

    if (--subscribers_ > 0)
        return;

    stop();
}

void Laser::subscribe()
{
    boost::mutex::scoped_lock lock(sub_lock_);

    if (0 == subscribers_++) {

        Status status = driver_->startStreams(Source_Lidar_Scan);
        if (Status_Ok != status)
            ROS_ERROR("Laser: failed to start laser stream: %s",
                      Channel::statusString(status));
    }
}
}
