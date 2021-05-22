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

    static cv::Vec3i hsv_threshold() {
      return {0, 0, 0};
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
  cv::Mat mask;
  cv::inRange(src, water_option::color_threshold(), cv::Vec3i {255, 255, 255}, mask);

  auto target_map = target_filter::apply(src);

  cv::bitwise_or(mask, target_map, mask);

  return smooth(mask);
}

cv::Mat &trs::water_filter::smooth(cv::Mat &in) {
  auto neigh_count = [&in](auto x, auto y){
    auto count = 0u;
    for (auto i : {-1, 0, 1})
      for (auto j : {-1, 0, 1}){
        if (!(i == 0 && j == 0))
          if (in.at<int>(x + i, y + j) != 0)
            ++count;
      }
    return count;
  };
  for (std::weakly_incrementable auto i : std::views::iota(1, in.rows))
    for (std::weakly_incrementable auto j : std::views::iota(1, in.cols)){
      if (in.at<int>(i, j) == 0){
        in.at<int>(i, j) = (neigh_count(i, j) >= 5) ? 255 : 0;
      }
    }
  return in;
}
