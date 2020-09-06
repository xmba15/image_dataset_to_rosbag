/**
 * @file    stereo_bag_converter.cpp
 *
 * @author  btran
 *
 */

#include <image_dataset_to_rosbag/BasicUtility.hpp>
#include <image_dataset_to_rosbag/Constants.hpp>
#include <image_dataset_to_rosbag/PerceptionUtility.hpp>
#include <image_dataset_to_rosbag/TypeConversions.hpp>

#include "stereo_bag_converter.hpp"

namespace perception
{
const std::unordered_map<StereoBagConverter::LenSide, std::string> StereoBagConverter::LEN_SIDE_STRING_MAP = {
    {LenSide::LEFT, "left"}, {LenSide::RIGHT, "right"}};

StereoBagConverter::~StereoBagConverter()
{
}

StereoBagConverter::StereoBagConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : BagConverterBase(nh, pnh)
    , m_lenSideParams({{LenSide::LEFT, LenSideParam()}, {LenSide::RIGHT, LenSideParam()}})
    , m_cinfo(nullptr)
    , m_ci({{LenSide::LEFT, nullptr}, {LenSide::RIGHT, nullptr}})
    , m_imageNames({{LenSide::LEFT, {}}, {LenSide::RIGHT, {}}})
    , m_cameraModels({{LenSide::LEFT, image_geometry::PinholeCameraModel()},
                      {LenSide::RIGHT, image_geometry::PinholeCameraModel()}})
{
    pnh.param<std::string>("camera_name", m_commonParam.cameraName, "");
    pnh.param<std::string>("time_stamp_path", m_commonParam.timeStampPath, "");
    pnh.param<std::string>("image_encoding", m_commonParam.imageEncoding, "");
    pnh.param<double>("frame_rate", m_commonParam.frameRate, 10);
    pnh.param<std::string>("bag_file_path", m_commonParam.bagFilePath, "output.bag");
    pnh.param<bool>("already_rectified", m_commonParam.alreadyRectified, false);
    pnh.param<bool>("save_rectified", m_commonParam.saveRectified, true);

    utils::validateParam(m_commonParam);

    m_cinfo.reset(new camera_info_manager::CameraInfoManager(nh));
    m_cinfo->setCameraName(m_commonParam.cameraName);

    this->initLenSideParam(nh, pnh, LenSide::LEFT);
    this->initLenSideParam(nh, pnh, LenSide::RIGHT);
    if (m_imageNames[LenSide::LEFT].size() != m_imageNames[LenSide::RIGHT].size()) {
        throw std::runtime_error("numbers of image for left lens and right lens mismatch\n");
    }

    this->initTimeStamps();

    ROS_INFO_STREAM_ONCE("Parsing " << m_imageNames[LenSide::LEFT].size() << " images...");

    m_bag.open(m_commonParam.bagFilePath, rosbag::bagmode::Write);

    while (this->writeToBag()) {
    }
}

void StereoBagConverter::initLenSideParam(ros::NodeHandle& nh, ros::NodeHandle& pnh, const LenSide lenSide)
{
    std::string curLenName = LEN_SIDE_STRING_MAP.at(lenSide);
    pnh.param<std::string>(curLenName + "_image_list_path", m_lenSideParams[lenSide].imageListPath, "");
    pnh.param<std::string>(curLenName + "_image_path", m_lenSideParams[lenSide].imagePath, "");
    pnh.param<std::string>(curLenName + "_calib_path", m_lenSideParams[lenSide].calibPath, "");
    pnh.param<std::string>(curLenName + "_image_topic_raw", m_lenSideParams[lenSide].imageTopicRaw,
                           m_commonParam.cameraName + "/" + curLenName + "/image_raw");
    pnh.param<std::string>(curLenName + "_image_topic_rect", m_lenSideParams[lenSide].imageTopicRect,
                           m_commonParam.cameraName + "/" + curLenName + "/image_rect");
    pnh.param<std::string>(curLenName + "_camera_info_topic", m_lenSideParams[lenSide].cameraInfoTopic,
                           m_commonParam.cameraName + "/" + curLenName + "/camera_info");

    utils::validateParam(m_lenSideParams[lenSide]);
    m_cinfo->loadCameraInfo("file://" + m_lenSideParams[lenSide].calibPath);
    m_ci[lenSide].reset(new sensor_msgs::CameraInfo(m_cinfo->getCameraInfo()));
    m_imageNames[lenSide] = utils::parseMetaDataFile(m_lenSideParams[lenSide].imageListPath);
    if (m_commonParam.saveRectified) {
        if (!m_cameraModels[lenSide].fromCameraInfo(m_ci[lenSide])) {
            throw std::runtime_error("failed to initialize camera model on " + curLenName + " len\n");
        }
        m_rectificationMaps[lenSide] = getRectificationMap(m_cameraModels[lenSide]);
    }
}

bool StereoBagConverter::writeToBag()
{
    if (m_counter == m_imageNames[LenSide::LEFT].size()) {
        return false;
    }

    ROS_INFO_STREAM("Writing " << m_counter << " images...");

    const auto& timeStampRos = m_timeStamps[m_counter];

    for (const LenSide lenSide : {LenSide::LEFT, LenSide::RIGHT}) {
        const std::string curImagePath = m_lenSideParams[lenSide].imagePath + "/" + m_imageNames[lenSide][m_counter];
        const cv::Mat curImage = cv::imread(curImagePath, -1);
        if (curImage.empty()) {
            ROS_WARN_STREAM("failed to open " + curImagePath);
            return false;
        }
        sensor_msgs::ImagePtr imageMsg = toImageMsg(curImage, m_commonParam.imageEncoding);
        imageMsg->header.stamp = timeStampRos;
        imageMsg->header.frame_id = m_commonParam.cameraName;
        imageMsg->header.seq = m_counter;
        m_ci[lenSide]->header = imageMsg->header;

        if (m_commonParam.alreadyRectified) {
            m_bag.write(m_lenSideParams[lenSide].imageTopicRect, timeStampRos, imageMsg);
        } else {
            m_bag.write(m_lenSideParams[lenSide].imageTopicRaw, timeStampRos, imageMsg);
            if (m_commonParam.saveRectified) {
                cv::Mat rectImage;
                cv::remap(curImage, rectImage, m_rectificationMaps[lenSide].first, m_rectificationMaps[lenSide].second,
                          cv::INTER_CUBIC);
                sensor_msgs::ImagePtr rectImageMsg = toImageMsg(rectImage, m_commonParam.imageEncoding);
                rectImageMsg->header.stamp = timeStampRos;
                rectImageMsg->header.frame_id = m_commonParam.cameraName;
                rectImageMsg->header.seq = m_counter;
                m_bag.write(m_lenSideParams[lenSide].imageTopicRect, timeStampRos, rectImageMsg);
            }
        }
        m_bag.write(m_lenSideParams[lenSide].cameraInfoTopic, timeStampRos, m_ci[lenSide]);
    }

    m_counter++;
    return true;
}

void StereoBagConverter::initTimeStamps()
{
    if (!m_commonParam.timeStampPath.empty()) {
        ROS_INFO_STREAM("Init time stamp with provided time stamp data");
        auto timeStampsNs = utils::parseMetaDataFile(m_commonParam.timeStampPath);
        if (m_imageNames[LenSide::LEFT].size() != timeStampsNs.size()) {
            throw std::runtime_error("number of image size and time stamp size mismatch\n");
        }
        m_timeStamps = this->getTimeStamps(timeStampsNs);
    } else {
        ROS_INFO_STREAM("Init time stamp with frame rate");
        m_timeStamps =
            this->getTimeStamps(ros::Time::now(), m_commonParam.frameRate, m_imageNames[LenSide::LEFT].size());
    }
}
}  // namespace perception

namespace utils
{
template <> void validateParam(const perception::StereoBagConverter::CommonParam& params)
{
    if (params.alreadyRectified && !params.saveRectified) {
        throw ParamConfigError("we only have rectified images");
    }

    if (perception::STRING_TO_CV_ENCODING.find(params.imageEncoding) == perception::STRING_TO_CV_ENCODING.end()) {
        throw ParamConfigError("not supported image encoding: " + params.imageEncoding);
    }

    if (params.bagFilePath.empty()) {
        throw ParamConfigError("bag file path cannot be empty");
    }
}

template <> void validateParam(const perception::StereoBagConverter::LenSideParam& params)
{
    if (params.imageListPath.empty()) {
        throw ParamConfigError("image list path cannot be empty");
    }
    if (!fileExists(params.imageListPath)) {
        throw ParamConfigError("image list path does not exist: " + params.imageListPath);
    }

    if (params.imagePath.empty()) {
        throw ParamConfigError("image path cannot be empty");
    }
    if (!directoryExists(params.imagePath)) {
        throw ParamConfigError("image path does not exist: " + params.imagePath);
    }

    if (params.calibPath.empty()) {
        throw ParamConfigError("calib path cannot be empty");
    }

    if (!fileExists(params.calibPath)) {
        throw ParamConfigError("calib path does not exist: " + params.calibPath);
    }

    if (params.imageTopicRaw.empty()) {
        throw ParamConfigError("image raw topic cannot be empty");
    }

    if (params.imageTopicRect.empty()) {
        throw ParamConfigError("image rect topic cannot be empty");
    }

    if (params.cameraInfoTopic.empty()) {
        throw ParamConfigError("camera info topic cannot be empty");
    }
}
}  // namespace utils
