//
// Created by yalavrinenko on 22.05.2021.
//

#include <map/map.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv){
  auto test_path = "../../demo/16.jpg";

  trs::map map(test_path);
  auto img = map.raw_image();
  auto road = map.extract_roads();

  auto checks = map.checks();
  std::ranges::for_each(checks, [&img](auto const& check){
    auto position = cv::Point_<size_t>{check.position.first, check.position.second};
    cv::circle(img, position, 10, cv::Scalar{255}, 3);
    cv::putText(img, std::to_string(check.uid), position, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{128}, 3);
  });

  using namespace std::string_literals;
  auto raw_window = test_path  + " raw"s;
  auto road_window = test_path  + " road"s;

  auto scale = [](auto const& img, auto fx, auto fy) {
    cv::Mat out;
    cv::resize(img, out,{}, fx, fy);
    return out;
  };

  cv::imshow(raw_window, scale(img, 0.6, 0.6));
  cv::imshow(road_window, scale(road, 0.5, 0.5));

  cv::waitKey();
  return 0;
}