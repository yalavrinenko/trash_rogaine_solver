//
// Created by yalavrinenko on 22.05.2021.
//

#include "water_filter.hpp"
#include <opencv2/opencv.hpp>
#include "target_filter.hpp"

namespace {
  class water_option{
  public:
    static cv::Vec3i color_threshold() {
      return {190, 0, 0};
    }

    static std::pair<cv::Vec3i, cv::Vec3i> hsv_limits() {
      return {{90, 20, 200}, {100, 70, 255}};
      //return {{200, 50, 50}, {255, 100, 100}};
    }

    static unsigned grayscale_threshold() {
      return 170;
    }

    static cv::Vec3i system_color(){
      return {0, 0, 220};
    };
  };
}

cv::Mat trs::water_filter::apply(const cv::Mat &src) {
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, water_option::hsv_limits().first, water_option::hsv_limits().second, mask);

//  auto target_map = target_filter::apply(src);
//
//  cv::bitwise_or(mask, target_map, mask);

  return smooth(mask);
}

cv::Mat trs::water_filter::smooth(const cv::Mat &in) {
  cv::Mat_<double> kernel (5, 5);
  kernel << 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 1;
  kernel /= 25.0;

  cv::Mat smoothed;
  cv::filter2D(in, smoothed, 0, kernel);
  cv::inRange(smoothed, 10, 255, smoothed);

//  cv::filter2D(smoothed, smoothed, 0, kernel);
//  cv::inRange(smoothed, 10, 255, smoothed);
  return smoothed;
}
