/**
 * @file color_laser.cpp
 *
 * Copyright 2014
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

#include <arpa/inet.h>

#include <multisense_ros/color_laser.h>

//
// Anonymous namespace for locally scoped symbols
namespace {

    const uint32_t laser_cloud_step = 16;

} // namespace

namespace multisense_ros {

ColorLaser::ColorLaser(
    ros::NodeHandle& nh
):
    color_image_(),
    camera_info_(),
    color_laser_pointcloud_(),
    color_laser_publisher_(),
    color_image_sub_(),
    laser_pointcloud_sub_(),
    camera_info_sub_(),
    node_handle_(nh),
    data_lock_(),
    image_channels_(3)
{
    //
    // Initialize point cloud structure

    color_laser_pointcloud_.is_bigendian    = (htonl(1) == 1);
    color_laser_pointcloud_.is_dense        = true;
    color_laser_pointcloud_.height          = 1;
    color_laser_pointcloud_.point_step      = laser_cloud_step;

    color_laser_pointcloud_.fields.resize(4);
    color_laser_pointcloud_.fields[0].name     = "x";
    color_laser_pointcloud_.fields[0].offset   = 0;
    color_laser_pointcloud_.fields[0].count    = 1;
    color_laser_pointcloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    color_laser_pointcloud_.fields[1].name     = "y";
    color_laser_pointcloud_.fields[1].offset   = 4;
    color_laser_pointcloud_.fields[1].count    = 1;
    color_laser_pointcloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    color_laser_pointcloud_.fields[2].name     = "z";
    color_laser_pointcloud_.fields[2].offset   = 8;
    color_laser_pointcloud_.fields[2].count    = 1;
    color_laser_pointcloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    color_laser_pointcloud_.fields[3].name     = "rgb";
    color_laser_pointcloud_.fields[3].offset   = 12;
    color_laser_pointcloud_.fields[3].count    = 1;
    color_laser_pointcloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    color_laser_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_points2_color",
                                                                   10,
                                                                   boost::bind(&ColorLaser::startStreaming, this),
                                                                   boost::bind(&ColorLaser::stopStreaming, this));

}

ColorLaser::~ColorLaser()
{
}

void ColorLaser::colorImageCallback(
    const sensor_msgs::Image::ConstPtr& message
)
{
    boost::mutex::scoped_lock lock(data_lock_);

    //
    // Images are assumed to be 8 bit.

    image_channels_ = message->data.size() / (message->height * message->width);

    if (image_channels_ > 3)
    {
        ROS_ERROR("Unsupported number of color channels: %d", image_channels_);
    }

    color_image_ = *message;
}

void ColorLaser::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& message
)
{
    boost::mutex::scoped_lock lock(data_lock_);

    camera_info_ = *message;
}

void ColorLaser::laserPointCloudCallback(
    sensor_msgs::PointCloud2::Ptr message
)
{
    boost::mutex::scoped_lock lock(data_lock_);

    //
    // Make sure the associated camera_info/color image is within 2 seconds
    // of the current point cloud message

    if (message->header.stamp - color_image_.header.stamp > ros::Duration(2) ||
        message->header.stamp - camera_info_.header.stamp > ros::Duration(2))
    {
        return;
    }

    color_laser_pointcloud_.header = message->header;

    //
    // Here we assume that our the sizeof our intensity field is the same as
    // the sizeof our new rgb color field.

    color_laser_pointcloud_.data.resize(message->data.size());
    float* colorPointCloudDataP = reinterpret_cast<float*>(&(color_laser_pointcloud_.data[0]));

    //
    // Iterate over all the points in the point cloud, Use the camera projection
    // matrix to project each point into the left camera image,
    // colorize the point with the corresponding image point in the left
    // camera. If the point does not project into the camera image
    // do not add it to the point cloud.

    float* pointCloudDataP = reinterpret_cast<float*>(&(message->data[0]));

    uint32_t height = message->height;
    uint32_t width = message->width;
    uint32_t cloudStep = message->point_step / message->fields.size();

    uint32_t validPoints = 0;
    for( uint32_t index = 0 ; index < height * width ;
            ++index, pointCloudDataP += cloudStep)
    {
        float x = pointCloudDataP[0];
        float y = pointCloudDataP[1];
        float z = pointCloudDataP[2];

        //
        // Invalid points from the laser will have a distance of 60m.
        // Since these points have the laser calibration applied to them
        // add in a 2m buffer for filtering out invalid points

        if (sqrt(pow(x,2) + pow(y,2) + pow(z,2)) > 58.0)
        {
            continue;
        }


        //
        // Compute the (u,v) camera coordinates corresponding to our laser
        // point

        //
        // (fx*x + cx*z)/z
        uint32_t u = (camera_info_.P[0] * x + camera_info_.P[2] * z) / z;
        //
        // (fy*y + cy*z)/z
        uint32_t v = (camera_info_.P[5] * y + camera_info_.P[6] * z) / z;



        //
        // If our computed (u, v) point projects into the image use its
        // color value and add it to the color pointcloud message

        if (u < color_image_.width && v < color_image_.height && u >= 0 && v >= 0)
        {

            colorPointCloudDataP[0] = x;
            colorPointCloudDataP[1] = y;
            colorPointCloudDataP[2] = z;

            uint8_t* colorChannelP = reinterpret_cast<uint8_t*>(&colorPointCloudDataP[3]);

            //
            // Image data is assumed to be BRG and stored continuously in memory
            // both of which are the case for color images from the MultiSense

            uint8_t* imageDataP = &color_image_.data[(image_channels_ * v * color_image_.width) + (image_channels_ * u)];

            switch(image_channels_)
            {
                case 3:
                    colorChannelP[2] = imageDataP[2];
                case 2:
                    colorChannelP[1] = imageDataP[1];
                case 1:
                    colorChannelP[0] = imageDataP[0];
            }

            //
            // If we are dealing with a grayscale image then copy
            // the intensity value to all 3 channels

            if (image_channels_ == 1)
            {
                colorChannelP[2] = imageDataP[0];
                colorChannelP[1] = imageDataP[0];
            }

            colorPointCloudDataP += cloudStep;
            ++validPoints;
        }

    }

    color_laser_pointcloud_.data.resize(validPoints * laser_cloud_step);
    color_laser_pointcloud_.width = validPoints;
    color_laser_pointcloud_.row_step = validPoints * laser_cloud_step;

    color_laser_publisher_.publish(color_laser_pointcloud_);
}

void ColorLaser::startStreaming()
{
    if (color_image_sub_.getNumPublishers() == 0 &&
        camera_info_sub_.getNumPublishers() == 0 &&
        laser_pointcloud_sub_.getNumPublishers() == 0)
    {
        color_image_sub_      = node_handle_.subscribe("image_rect_color",
                                                      10,
                                                      &multisense_ros::ColorLaser::colorImageCallback,
                                                      this);

        camera_info_sub_      = node_handle_.subscribe("camera_info",
                                                      10,
                                                      &multisense_ros::ColorLaser::cameraInfoCallback,
                                                      this);

        laser_pointcloud_sub_ = node_handle_.subscribe("lidar_points2",
                                                      10,
                                                      &multisense_ros::ColorLaser::laserPointCloudCallback,
                                                      this);
    }

}

void ColorLaser::stopStreaming()
{
    if (color_laser_publisher_.getNumSubscribers() == 0)
    {
        color_image_sub_.shutdown();
        camera_info_sub_.shutdown();
        laser_pointcloud_sub_.shutdown();
    }
}

} // namespace

int main(int argc, char** argv)
{

    try
    {
        ros::init(argc, argv, "color_laser_publisher");

        ros::NodeHandle nh;

        multisense_ros::ColorLaser colorLaserPublisher(nh);

        ros::spin();
    }
    catch(std::exception& e)
    {
        ROS_ERROR("%s", e.what());
        return 1;
    }

    return 0;
}


