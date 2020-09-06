/**
 * @file    bag_converter_base.cpp
 *
 * @author  btran
 *
 */

#include "bag_converter_base.hpp"

#include <image_dataset_to_rosbag/TypeConversions.hpp>

namespace perception
{
BagConverterBase::BagConverterBase(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_counter(0)
{
}

BagConverterBase::~BagConverterBase()
{
    m_bag.close();
}

std::vector<ros::Time> BagConverterBase::getTimeStamps(const ros::Time& baseTime, const double frameRate,
                                                       const int size) const
{
    std::vector<ros::Time> timeStamps;
    timeStamps.reserve(size);
    ros::Duration interval = ros::Duration(ros::Rate(frameRate));

    for (int i = 0; i < size; ++i) {
        timeStamps.emplace_back(baseTime + interval * i);
    }

    return timeStamps;
}

std::vector<ros::Time> BagConverterBase::getTimeStamps(const std::vector<std::string>& timeStampsNs) const
{
    if (timeStampsNs.empty()) {
        ROS_WARN("time stamps empty");
        return {};
    }

    std::vector<ros::Time> timeStamps;
    timeStamps.reserve(timeStamps.size());
    std::transform(
        timeStampsNs.begin(), timeStampsNs.end(), std::back_inserter(timeStamps),
        [](const std::string& curTimeStampNs) { return perception::toRosTime(std::atoi(curTimeStampNs.c_str())); });

    return timeStamps;
}
}  // namespace perception
