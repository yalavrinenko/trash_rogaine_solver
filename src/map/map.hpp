//
// Created by yalavrinenko on 22.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_MAP_HPP
#define TRASH_ROGAINE_SOLVER_MAP_HPP

#include "ocv_image_loader.hpp"
#include "filters/water_filter.hpp"
#include <utils/concepts.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <map/filters/target_filter.hpp>
#include <map/detectors/trivial_cp_detector.hpp>

namespace trs {
  class map_holder {
  public:
    explicit map_holder(cv::Mat image): image_{std::move(image)} {}

    auto& image() { return image_; }
    [[nodiscard]] auto const& image() const { return image_; }

    [[nodiscard]] size_t width() const { return image_.cols; }
    [[nodiscard]] size_t height() const { return image_.rows; }

  private:
    cv::Mat image_;
  };

  template <
      MapHolder holding_strategy = map_holder,
      ImageLoader load_map_strategy = ocv_image_loader,
      Filter road_selection_strategy = water_filter,
      CheckPointDetector check_point_detection_strategy = trivial_cp_detector<target_filter>
          >
  class map {
  public:
    using holder = holding_strategy;
    using loader = load_map_strategy;
    using road_extractor = road_selection_strategy;
    using cp_detector = check_point_detection_strategy;

    explicit map(std::filesystem::path const& img, loader load_strategy={}, cp_detector detector = {}){
      handle_ = std::make_unique<holder>(load_strategy.load(img));
      check_points_ = detector.extract_check_points(handle_->image());
    }

    decltype(auto) raw_image() const { return handle_->image(); }

    cv::Mat const& extract_roads(road_extractor extractor={}) {
      roads_ = std::make_unique<holder>(extractor.apply(handle_->image()));
      return roads_->image();
    }

    [[nodiscard]] cv::Mat const& extract_roads() const {
      return roads_->image();
    }

    auto const& checks() const { return check_points_; }

  private:
    std::unique_ptr<holder> handle_;
    std::unique_ptr<holder> roads_;
    std::unordered_map<size_t, check_point> check_points_;
  };
}


#endif //TRASH_ROGAINE_SOLVER_MAP_HPP
