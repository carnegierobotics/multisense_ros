/**
 * @file laser.cpp
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

#include <multisense_ros/laser.h>
#include <multisense_ros/RawLidarData.h>
#include <multisense_ros/RawLidarCal.h>
#include <angles/angles.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <arpa/inet.h>

using namespace crl::multisense;

namespace { // anonymous

bool computePostSpindleCal(const KDL::Tree&  tree,
                           const KDL::Frame& spindle_T_laser,
                           KDL::Frame&       post_spindle_cal)
{
    KDL::Chain chain;
    if (!tree.getChain("hokuyo_link", "head_hokuyo_frame", chain)) {
        ROS_ERROR("Laser: error extracting post-spindle chain from KDL tree");
        return false;
    }

    if (chain.getNrOfJoints() != 0) {
        ROS_ERROR("Laser: expected 0 joints in chain. Got %u\n", 
                  chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive fksolver(chain);

    KDL::Frame scan_post_spindle_cal_T_hokuyo_head;
    if(fksolver.JntToCart(joint_pos, scan_post_spindle_cal_T_hokuyo_head) < 0) {
        ROS_ERROR("Laser: error in FK for post-spindle calcs");
        return false;
    }

    KDL::Frame nominal_T_optical(KDL::Rotation::RPY(M_PI/2, 
                                                    -M_PI/2, 0.0).Inverse(), 
                                 KDL::Vector());
    post_spindle_cal = (nominal_T_optical * spindle_T_laser * 
                        (scan_post_spindle_cal_T_hokuyo_head * 
                         nominal_T_optical).Inverse());

    return true;
}

bool computePreSpindleCal(const KDL::Tree&  tree,
                          const KDL::Frame& camera_T_spindle_fixed,
                          KDL::Frame&       pre_spindle_cal)
{
    KDL::Chain head_spindle_chain;
    if (!tree.getChain("head", "pre_spindle", head_spindle_chain)) {
        ROS_ERROR("Laser: error extracting head-spindle chain from KDL tree");
        return false;
    }

    KDL::Chain head_cam_chain;
    if (!tree.getChain("head", "left_camera_optical_frame", head_cam_chain)) {
        ROS_ERROR("Laser: error extracting head-camera chain from KDL tree");
        return false;
    }

    if (head_cam_chain.getNrOfJoints() != 0) {
        ROS_ERROR("Laser: expected 0 joints in head-camera chain. Got %u\n", 
                  head_cam_chain.getNrOfJoints());
        return false;
    }

    if (head_spindle_chain.getNrOfJoints() != 0) {
        ROS_ERROR("Laser: expected 0 joints in head-spindle chain. Got %u\n", 
                  head_spindle_chain.getNrOfJoints());
        return false;
    }

    KDL::JntArray joint_pos(0);
    KDL::ChainFkSolverPos_recursive head_spindle_solver(head_spindle_chain);
    KDL::ChainFkSolverPos_recursive head_cam_solver(head_cam_chain);
    KDL::Frame head_T_pre_spindle;
    KDL::Frame head_T_cam_optical;
    if(head_spindle_solver.JntToCart(joint_pos, head_T_pre_spindle) < 0) {
        ROS_ERROR("Laser: error in FK for head_spindle calcs");
        return false;
    }

    if(head_cam_solver.JntToCart(joint_pos, head_T_cam_optical) < 0) {
        ROS_ERROR("Laser: error in FK for head_cam calcs");
        return false;
    }

    KDL::Frame pre_spindle_T_pre_spindle_rot(KDL::Rotation::RPY(M_PI/2, 
                                                                -M_PI/2, 0.0).Inverse(), 
                                             KDL::Vector());

    KDL::Frame cam_opt_T_pre_spindle = (head_T_cam_optical.Inverse() * 
                                        head_T_pre_spindle);

    pre_spindle_cal = (cam_opt_T_pre_spindle.Inverse()
                       * camera_T_spindle_fixed * 
                       pre_spindle_T_pre_spindle_rot.Inverse());

    return true;
}


bool computeCal(const std::string& robot_desc_string,
                const KDL::Frame&  spindle_T_laser, 
                const KDL::Frame&  camera_T_spindle_fixed,
                KDL::Frame&        pre_spindle_cal, 
                KDL::Frame&        post_spindle_cal)
{
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
        ROS_ERROR("Laser: error parsing robot description using kdl_parser");
        return false;
    }

    bool success;
    success = computePostSpindleCal(tree, spindle_T_laser, post_spindle_cal);
    if (!success) {
        ROS_ERROR("Laser: error calculating post spindle calibration");
        return false;
    }

    success = computePreSpindleCal(tree, camera_T_spindle_fixed, pre_spindle_cal);
    if (!success) {
        ROS_ERROR("Laser: error calculating pre spindle calibration");
        return false;
    }

    return true;
}

KDL::Frame makeFrame(float T[4][4])
{
    KDL::Frame out;

    for (int i=0; i < 3; i++) {
        for (int j=0; j < 3; j++)
            out.M(i,j) = T[i][j];
    }

    out.p[0] = T[0][3];
    out.p[1] = T[1][3];
    out.p[2] = T[2][3];

    return out;
}

void getLaserCal(lidar::Calibration& c,
                 const std::string&  robot_desc,
                 KDL::Frame&         pre_spindle_cal,
                 KDL::Frame&         post_spindle_cal)
{   
    KDL::Frame spindle_T_laser = makeFrame(c.laserToSpindle).Inverse();
    KDL::Frame camera_T_spindle_fixed = makeFrame(c.cameraToSpindleFixed);

    bool success;
    success = computeCal(robot_desc,
                         spindle_T_laser, camera_T_spindle_fixed,
                         pre_spindle_cal, post_spindle_cal);
    if (!success)
        ROS_ERROR("Laser: error computing lidar calibration.");
}

void pushCal(sensor_msgs::JointState& msg, 
	     const std::string&       name, 
	     double                   pos)
{
    msg.name.push_back(name);
    msg.position.push_back(pos);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
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
             const std::string& robot_desc)
    : driver_(driver),
      subscribers_(0)
{
    ros::NodeHandle nh("laser");

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

    //
    // Get frame_id

    nh.param("frame_id", frame_id_, std::string("/head_hokuyo_frame"));

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
        // Calibration for laser scan topic

        getLaserCal(lidar_cal_, robot_desc, scan_pre_spindle_cal_, scan_post_spindle_cal_);

        //
        // Calibration for point cloud topic

        pc_post_spindle_cal_  = makeFrame(lidar_cal_.laserToSpindle);
        pc_pre_spindle_cal_ = makeFrame(lidar_cal_.cameraToSpindleFixed);        
    }

    //
    // Default joint message for LaserScan message

    js_msg_.name.push_back("hokuyo_joint");
    js_msg_.position.push_back(0.0);
    js_msg_.velocity.push_back(0.0);
    js_msg_.effort.push_back(0.0);

    double roll, pitch, yaw;
    scan_pre_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    pushCal(js_msg_, "pre_spindle_cal_x_joint", scan_pre_spindle_cal_.p[0]);
    pushCal(js_msg_, "pre_spindle_cal_y_joint", scan_pre_spindle_cal_.p[1]);
    pushCal(js_msg_, "pre_spindle_cal_z_joint", scan_pre_spindle_cal_.p[2]);
    pushCal(js_msg_, "pre_spindle_cal_roll_joint",  roll);
    pushCal(js_msg_, "pre_spindle_cal_pitch_joint", pitch);
    pushCal(js_msg_, "pre_spindle_cal_yaw_joint",   yaw);

    scan_post_spindle_cal_.M.GetRPY(roll, pitch, yaw);
    pushCal(js_msg_, "post_spindle_cal_x_joint", scan_post_spindle_cal_.p[0]);
    pushCal(js_msg_, "post_spindle_cal_y_joint", scan_post_spindle_cal_.p[1]);
    pushCal(js_msg_, "post_spindle_cal_z_joint", scan_post_spindle_cal_.p[2]);
    pushCal(js_msg_, "post_spindle_cal_roll_joint",  roll);
    pushCal(js_msg_, "post_spindle_cal_pitch_joint", pitch);
    pushCal(js_msg_, "post_spindle_cal_yaw_joint",   yaw);

    //
    // Joint publisher

    ros::NodeHandle nh_js("laser_joint");
    js_pub_ = nh_js.advertise<sensor_msgs::JointState>("/joint_states", 40);

    //
    // Create scan publisher

    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 20,
                boost::bind(&Laser::subscribe, this),
                boost::bind(&Laser::unsubscribe, this));

    //
    // Create point cloud publisher

    point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points2", 5,
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

    const uint32_t cloud_step = 16;

    point_cloud_.data.resize(cloud_step * header.pointCount);

    if (4 != point_cloud_.fields.size()) {

        point_cloud_.is_bigendian    = (htonl(1) == 1);
        point_cloud_.is_dense        = true;
        point_cloud_.point_step      = cloud_step;
        point_cloud_.height          = 1;
        point_cloud_.header.frame_id = "/left_camera_optical_frame";

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
        point_cloud_.fields[3].datatype = sensor_msgs::PointField::UINT32;
    }

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

    for(uint32_t i=0; i<header.pointCount; ++i, cloudP += cloud_step) {
        
        //
        // Percent through the scan arc

        const double percent = static_cast<double>(i) / static_cast<double>(header.pointCount - 1);

        //
        // The mirror angle for this point, invert for mirror motor direction

        const double mirrorTheta = -1.0 * (mirrorThetaStart + percent * arcRadians);

        //
        // The rotation about the spindle

        const double spindleTheta    = spindleAngleStart + percent * spindleAngleRange;
        const double cosSpindleTheta = std::cos(spindleTheta);
        const double sinSpindleTheta = std::sin(spindleTheta);

        const KDL::Rotation spindleFromMotor(cosSpindleTheta, -sinSpindleTheta, 0.0,
                                             sinSpindleTheta,  cosSpindleTheta, 0.0,
                                             0.0,              0.0,             1.0);
        //
        // The coordinate in left optical frame

        const double      rangeMeters = 1e-3 * static_cast<double>(header.rangesP[i]);  // from millimeters
        const KDL::Vector pointMotor  = (pc_post_spindle_cal_ * 
                                         KDL::Vector(rangeMeters * -std::sin(mirrorTheta), 0.0,
                                                     rangeMeters *  std::cos(mirrorTheta)));
        const KDL::Vector pointCamera = pc_pre_spindle_cal_ * (spindleFromMotor * pointMotor);
        
        //
        // Copy data to point cloud structure

        const float xyz[3] = {static_cast<float>(pointCamera.x()),
                              static_cast<float>(pointCamera.y()),
                              static_cast<float>(pointCamera.z())};
                        
        memcpy(cloudP, &(xyz[0]), pointSize);
        memcpy((cloudP + pointSize), &(header.intensitiesP[i]), sizeof(uint32_t));
    }

    point_cloud_pub_.publish(point_cloud_);
}

void Laser::scanCallback(const lidar::Header& header)
{
    //
    // Get out if we have no work to do 

    if (!(0 == (header.scanId % 40)         ||
          scan_pub_.getNumSubscribers() > 0 ||
          raw_lidar_data_pub_.getNumSubscribers() > 0))
        return;

    //
    // The URDF assumes that the laser straight up at a joint angle of 0 degrees

    const float offset = M_PI;

    const ros::Time start_absolute_time = ros::Time(header.timeStartSeconds,
                                                    1000 * header.timeStartMicroSeconds);
    const ros::Time end_absolute_time   = ros::Time(header.timeEndSeconds,
                                                    1000 * header.timeEndMicroSeconds);
    const ros::Time scan_time((end_absolute_time - start_absolute_time).toSec());

    const float angle_start = 1e-6 * static_cast<float>(header.spindleAngleStart) - offset;
    const float angle_end   = 1e-6 * static_cast<float>(header.spindleAngleEnd) - offset;
    const float angle_diff  = angles::shortest_angular_distance(angle_start, angle_end);
    const float velocity    = angle_diff / scan_time.toSec();

    //
    // Publish joint state for beginning of scan

    js_msg_.header.frame_id = "";
    js_msg_.header.stamp    = start_absolute_time;
    js_msg_.position[0]     = angle_start;
    js_msg_.velocity[0]     = velocity;
    js_pub_.publish(js_msg_);

    //
    // Publish joint state for end of scan
    
    js_msg_.header.frame_id = "";
    js_msg_.header.stamp    = end_absolute_time;
    js_msg_.position[0]     = angle_end;
    js_msg_.velocity[0]     = velocity;
    js_pub_.publish(js_msg_);

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
