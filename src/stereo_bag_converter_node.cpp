/**
 * @file    stereo_bag_converter_node.cpp
 *
 * @author  btran
 *
 */

#include "stereo_bag_converter.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_bag_converter_converter");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    perception::StereoBagConverter obj(nh, pnh);

    ROS_INFO_STREAM("Finished writing to rosbag");
    ros::shutdown();

    return 0;
}
