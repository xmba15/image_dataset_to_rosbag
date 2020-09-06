/**
 * @file    PerceptionUtility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <utility>

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>

namespace perception
{
inline std::pair<cv::Mat, cv::Mat> getRectificationMap(const image_geometry::PinholeCameraModel& cameraModel)
{
    cv::Mat mapX;
    cv::Mat mapY;
    cv::initUndistortRectifyMap(cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(),
                                cameraModel.rotationMatrix(), cameraModel.projectionMatrix(),
                                cv::Size(cameraModel.cameraInfo().width, cameraModel.cameraInfo().height), CV_32FC1,
                                mapX, mapY);

    return std::make_pair(mapX, mapY);
}
}  // namespace perception
