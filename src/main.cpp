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

  using namespace std::string_literals;
  auto raw_window = test_path  + " raw"s;
  auto road_window = test_path  + " road"s;

  cv::namedWindow(raw_window);

  auto scale = [](auto const& img, auto fx, auto fy) {
    cv::Mat out;
    cv::resize(img, out,{}, fx, fy);
    return out;
  };

  cv::imshow(raw_window, scale(img, 0.5, 0.5));
  cv::imshow(road_window, scale(road, 1, 1));
  cv::waitKey();
  return 0;
}