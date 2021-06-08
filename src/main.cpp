//
// Created by yalavrinenko on 22.05.2021.
//

#include <map/map.hpp>
#include <iostream>
#include <string>
#include <pathfinders/Astar_searcher.hpp>

int main(int argc, char** argv){
  auto test_path = "../../demo/test.png";
  //auto test_path = "../../demo/16.jpg";

  trs::map map(test_path);
  auto img = map.raw_image();
  auto road = map.extract_roads();

  auto checks = map.checks();
  std::ranges::for_each(checks, [&img](auto const& check){
    auto position = cv::Point_<size_t>{check.second.position.first, check.second.position.second};
    cv::circle(img, position, 10, cv::Scalar{255}, 3);
    cv::putText(img, std::to_string(check.second.uid), position, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{128}, 3);
  });

  auto aster = trs::Astar_searcher(map);

  do{
    int from, to;
    std::cin >> from >> to;
    std::cout << "Calculating distance from " << from << " to " << to << std::endl;
    auto track = aster.find_path(checks[from].position, checks[to].position);

    std::cout << "Distance from " << from << " to " << to << " is: " << track.first << "\n";
    for (auto &tp : track.second) {
      cv::circle(img, cv::Point_<int>{tp.first, tp.second}, 2, cv::Scalar{255}, 2);
    }

    using namespace std::string_literals;
    auto raw_window = test_path + " raw"s;

    auto scale = [](auto const &img, auto fx, auto fy) {
      cv::Mat out;
      cv::resize(img, out, {}, fx, fy);
      return out;
    };

    cv::imshow(raw_window, scale(img, 0.6, 0.6));

//  auto road_window = test_path  + " road"s;
//  cv::imshow(road_window, scale(road, 0.5, 0.5));


  } while (cv::waitKey());
  return 0;
}