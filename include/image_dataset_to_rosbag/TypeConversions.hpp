/**
 * @file    TypeConversions.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace perception
{
inline ros::Time toRosTime(const uint64_t timestampNs)
{
    ros::Time timeStamp;
    timeStamp.fromNSec(timestampNs);
    return timeStamp;
}

inline sensor_msgs::ImagePtr toImageMsg(const cv::Mat& image, const std::string& encoding)
{
    return cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
}
}  // namespace perception
