/**
 * @file    bag_converter_base.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

namespace perception
{
class BagConverterBase
{
 protected:
    BagConverterBase(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~BagConverterBase();

    virtual bool writeToBag() = 0;
    virtual void initTimeStamps() = 0;

    std::vector<ros::Time> getTimeStamps(const ros::Time& baseTime, const double frameRate, const int size) const;
    std::vector<ros::Time> getTimeStamps(const std::vector<std::string>& timeStampsNs) const;

 protected:
    rosbag::Bag m_bag;
    std::vector<ros::Time> m_timeStamps;
    int m_counter;
};
}  // namespace perception
