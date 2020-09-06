/**
 * @file    stereo_bag_converter.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>

#include <image_dataset_to_rosbag/ParamConfig.hpp>

#include "bag_converter_base.hpp"

namespace perception
{
class StereoBagConverter : public BagConverterBase
{
 public:
    enum class LenSide : int { LEFT = 0, RIGHT = 1, MAX = RIGHT };

    struct CommonParam {
        std::string cameraName;
        std::string timeStampPath;
        std::string imageEncoding;
        double frameRate;
        std::string bagFilePath;
        bool alreadyRectified;  // flag to check if the input images are already rectified or not
        bool saveRectified;     // flag to choose to save rectified images or not
    };

    struct LenSideParam {
        std::string imageListPath;
        std::string imagePath;
        std::string calibPath;
        std::string imageTopicRaw;
        std::string imageTopicRect;
        std::string cameraInfoTopic;
    };

 public:
    StereoBagConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~StereoBagConverter();

 private:
    void initTimeStamps() final;
    bool writeToBag() final;

    void initLenSideParam(ros::NodeHandle& nh, ros::NodeHandle& pnh, const LenSide lenSide);

 private:
    static const std::unordered_map<LenSide, std::string> LEN_SIDE_STRING_MAP;

    CommonParam m_commonParam;
    std::unordered_map<LenSide, LenSideParam> m_lenSideParams;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfo;
    std::unordered_map<LenSide, sensor_msgs::CameraInfoPtr> m_ci;
    std::unordered_map<LenSide, std::vector<std::string>> m_imageNames;
    std::unordered_map<LenSide, image_geometry::PinholeCameraModel> m_cameraModels;
    std::unordered_map<LenSide, std::pair<cv::Mat, cv::Mat>> m_rectificationMaps;
};
}  // namespace perception

template <> void utils::validateParam(const perception::StereoBagConverter::CommonParam& params);
template <> void utils::validateParam(const perception::StereoBagConverter::LenSideParam& params);
