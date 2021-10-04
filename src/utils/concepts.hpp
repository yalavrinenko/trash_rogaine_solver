//
// Created by yalavrinenko on 22.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_CONCEPTS_HPP
#define TRASH_ROGAINE_SOLVER_CONCEPTS_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

namespace trs{
  using path = std::pair<double, std::vector<std::pair<int, int>>>;
  using check_point_vertex = std::pair<size_t, size_t>;

  struct check_point{
    std::pair<size_t, size_t> position;
    size_t uid;
  };

  struct solution {
    double distance;
    size_t score;
    std::vector<size_t> trace;
    std::vector<path> track;
  };
}

template <typename T>
concept ImageLoader = requires(T loader) {
  {loader.load(std::filesystem::path{}) } -> std::convertible_to<cv::Mat>;
};

template <typename T>
concept MapHolder = requires (T holder, T* pholder) {
  { T(cv::Mat{}) };
  { pholder->image() } -> std::convertible_to<cv::Mat> ;
  { pholder->width() } -> std::convertible_to<size_t>;
  { pholder->height() } -> std::convertible_to<size_t>;
};

template <typename T>
concept Filter = requires (T filter){
  {filter.apply(cv::Mat{})} -> std::convertible_to<cv::Mat>;
};

template <typename T>
concept CheckPointDetector = requires (T detector) {
  { detector.extract_check_points(cv::Mat{})};
};

template <typename T>
concept RogaineMap = requires (T map){
  {map.extract_roads()} -> std::convertible_to<cv::Mat>;
  {map.checks()} -> std::convertible_to<std::unordered_map<size_t, trs::check_point>>;
};

#endif //TRASH_ROGAINE_SOLVER_CONCEPTS_HPP
