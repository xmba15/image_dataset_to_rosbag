/**
 * @file    mono_bag_converter.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>

#include <image_dataset_to_rosbag/ParamConfig.hpp>

#include "bag_converter_base.hpp"

namespace perception
{
class MonoBagConverter : public BagConverterBase
{
 public:
    struct Param {
        std::string cameraName;
        std::string imageListPath;
        std::string imagePath;
        std::string calibPath;
        std::string timeStampPath;
        std::string imageEncoding;
        double frameRate;
        std::string bagFilePath;
        std::string imageTopic;
        std::string cameraInfoTopic;
    };

 public:
    MonoBagConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~MonoBagConverter();

 private:
    void initTimeStamps() final;
    bool writeToBag() final;

 private:
    Param m_param;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfo;
    sensor_msgs::CameraInfoPtr m_ci;

    std::vector<std::string> m_imageNames;
};
}  // namespace perception

template <> void utils::validateParam(const perception::MonoBagConverter::Param& params);
