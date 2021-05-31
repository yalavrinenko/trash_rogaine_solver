//
// Created by yalavrinenko on 23.05.2021.
//

#include "target_filter.hpp"
#include <opencv2/opencv.hpp>

namespace {
  class target_option{
  public:
    static cv::Vec3i system_color(){
      return {0, 0, 220};
    };

    static std::pair<cv::Scalar , cv::Scalar> hsv_limits_hi() {
      return {cv::Scalar {150.0, 190, 190}, {180.0, 255, 255}};
    }

    static std::pair<cv::Scalar , cv::Scalar> hsv_limits_lo() {
      return {cv::Scalar {0, 200, 200}, {10, 255, 255}};
    }
  };
}

cv::Mat trs::target_filter::apply(const cv::Mat &src) {
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  cv::Mat target_map_hi;
  cv::inRange(hsv, target_option::hsv_limits_hi().first, target_option::hsv_limits_hi().second, target_map_hi);

  cv::Mat target_map_lo;
  cv::inRange(hsv, target_option::hsv_limits_lo().first, target_option::hsv_limits_lo().second, target_map_lo);

  cv::bitwise_or(target_map_hi, target_map_lo, target_map_hi);
  return target_map_hi;
}
