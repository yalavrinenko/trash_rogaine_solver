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
  };
}

cv::Mat trs::target_filter::apply(const cv::Mat &src) {
  cv::Mat system_map;
  cv::inRange(src, target_option::system_color(), cv::Vec3i{70, 70, 255}, system_map);
  return system_map;
}
