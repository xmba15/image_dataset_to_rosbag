/**
 * @file    Constants.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <string>
#include <unordered_map>

#include <opencv2/opencv.hpp>

namespace perception
{
static const std::unordered_map<std::string, int> STRING_TO_CV_ENCODING = {{"mono8", CV_8UC1}, {"mono16", CV_16UC1},
                                                                           {"bgr8", CV_8UC3},  {"rgb8", CV_8UC3},
                                                                           {"bgra8", CV_8UC4}, {"rgb8a8", CV_8UC4}};

static constexpr float NS_TO_MICROS = 1e-3;
}  // namespace perception
