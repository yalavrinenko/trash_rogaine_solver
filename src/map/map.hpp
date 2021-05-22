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
      Filter road_selection_strategy = water_filter
          >
  class map {
  public:
    using holder = holding_strategy;
    using loader = load_map_strategy;
    using road_extractor = road_selection_strategy;

    explicit map(std::filesystem::path const& img, load_map_strategy load_strategy={}){
      handle_ = std::make_unique<holder>(load_strategy.load(img));
    }

    decltype(auto) raw_image() const { return handle_->image(); }

    cv::Mat const& extract_roads(road_extractor extractor={}) {
      roads_ = std::make_unique<holder>(extractor.apply(handle_->image()));
      return roads_map();
    }

  private:
    decltype(auto) roads_map() {
      if (!roads_)
        extract_roads();

      return roads_->image();
    }

    std::unique_ptr<holder> handle_;
    std::unique_ptr<holder> roads_;
  };
}


#endif //TRASH_ROGAINE_SOLVER_MAP_HPP
