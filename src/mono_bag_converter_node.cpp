/**
 * @file    mono_bag_converter_node.cpp
 *
 * @author  btran
 *
 */

#include "mono_bag_converter.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "mono_bag_converter_converter");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    perception::MonoBagConverter obj(nh, pnh);

    ROS_INFO_STREAM("Finished writing to rosbag");
    ros::shutdown();

    return 0;
}
