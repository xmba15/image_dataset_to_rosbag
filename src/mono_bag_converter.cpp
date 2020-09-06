/**
 * @file    mono_bag_converter.cpp
 *
 * @author  btran
 *
 */

#include <image_dataset_to_rosbag/BasicUtility.hpp>
#include <image_dataset_to_rosbag/Constants.hpp>
#include <image_dataset_to_rosbag/TypeConversions.hpp>

#include "mono_bag_converter.hpp"

namespace perception
{
MonoBagConverter::MonoBagConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : BagConverterBase(nh, pnh)
    , m_cinfo(new camera_info_manager::CameraInfoManager(nh))
    , m_ci(nullptr)
{
    pnh.param<std::string>("camera_name", m_param.cameraName, "");
    pnh.param<std::string>("image_list_path", m_param.imageListPath, "");
    pnh.param<std::string>("image_path", m_param.imagePath, "");
    pnh.param<std::string>("calib_path", m_param.calibPath, "");
    pnh.param<std::string>("time_stamp_path", m_param.timeStampPath, "");
    pnh.param<std::string>("image_encoding", m_param.imageEncoding, "");
    pnh.param<double>("frame_rate", m_param.frameRate, 10);
    pnh.param<std::string>("bag_file_path", m_param.bagFilePath, "output.bag");
    pnh.param<std::string>("image_topic", m_param.imageTopic, "/" + m_param.cameraName + "/image_raw");
    pnh.param<std::string>("camera_info_topic", m_param.cameraInfoTopic, "/" + m_param.cameraName + "/camera_info");

    utils::validateParam(m_param);

    m_cinfo->setCameraName(m_param.cameraName);
    m_cinfo->loadCameraInfo("file://" + m_param.calibPath);
    m_ci.reset(new sensor_msgs::CameraInfo(m_cinfo->getCameraInfo()));
    m_imageNames = utils::parseMetaDataFile(m_param.imageListPath);
    this->initTimeStamps();

    ROS_INFO_STREAM_ONCE("Parsing " << m_imageNames.size() << " images...");

    m_bag.open(m_param.bagFilePath, rosbag::bagmode::Write);

    while (this->writeToBag()) {
    }
}

MonoBagConverter::~MonoBagConverter()
{
}

void MonoBagConverter::initTimeStamps()
{
    if (!m_param.timeStampPath.empty()) {
        ROS_INFO_STREAM("Init time stamp with provided time stamp data");
        auto timeStampsNs = utils::parseMetaDataFile(m_param.timeStampPath);
        if (m_imageNames.size() != timeStampsNs.size()) {
            throw std::runtime_error("numbers of image size and time stamp size mismatch\n");
        }
        m_timeStamps = this->getTimeStamps(timeStampsNs);
    } else {
        ROS_INFO_STREAM("Init time stamp with frame rate");
        m_timeStamps = this->getTimeStamps(ros::Time::now(), m_param.frameRate, m_imageNames.size());
    }
}

bool MonoBagConverter::writeToBag()
{
    if (m_counter == m_imageNames.size()) {
        return false;
    }

    ROS_INFO_STREAM("Writing " << m_counter << " images...");

    const std::string curImagePath = m_param.imagePath + "/" + m_imageNames[m_counter];
    const cv::Mat curImage = cv::imread(curImagePath, -1);

    if (curImage.empty()) {
        ROS_WARN_STREAM("failed to open " + curImagePath);
        return false;
    }

    const auto& timeStampRos = m_timeStamps[m_counter];
    sensor_msgs::ImagePtr imageMsg = toImageMsg(curImage, m_param.imageEncoding);
    imageMsg->header.stamp = timeStampRos;
    imageMsg->header.frame_id = m_param.cameraName;
    imageMsg->header.seq = m_counter;
    m_ci->header = imageMsg->header;

    m_bag.write(m_param.imageTopic, timeStampRos, imageMsg);
    m_bag.write(m_param.cameraInfoTopic, timeStampRos, m_ci);

    m_counter++;
    return true;
}
}  // namespace perception

namespace utils
{
template <> void validateParam(const perception::MonoBagConverter::Param& params)
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

    if (perception::STRING_TO_CV_ENCODING.find(params.imageEncoding) == perception::STRING_TO_CV_ENCODING.end()) {
        throw ParamConfigError("not supported image encoding: " + params.imageEncoding);
    }

    if (params.bagFilePath.empty()) {
        throw ParamConfigError("bag file path cannot be empty");
    }

    if (params.imageTopic.empty()) {
        throw ParamConfigError("image topic cannot be empty");
    }

    if (params.cameraInfoTopic.empty()) {
        throw ParamConfigError("camera info topic cannot be empty");
    }
}
}  // namespace utils
