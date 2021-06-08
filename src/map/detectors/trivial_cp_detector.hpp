//
// Created by yalavrinenko on 23.05.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_TRIVIAL_CP_DETECTOR_HPP
#define TRASH_ROGAINE_SOLVER_TRIVIAL_CP_DETECTOR_HPP
#include <utils/concepts.hpp>
#include <unordered_map>

namespace trs{
  struct check_point{
    std::pair<size_t, size_t> position;
    size_t uid;
  };

  class general_detector{
  public:
    general_detector();
    std::unordered_map<size_t, check_point> extract_point(cv::Mat const& filtered);
    ~general_detector();
  private:
    std::unordered_map<size_t, check_point> get_check_point(cv::Mat const& frame, std::vector<cv::Vec3f> const& circles);
    static cv::Mat smooth(cv::Mat const &in, int iteration);
    std::string digit_recognition(cv::Mat const& frame);

    struct dnn_core;
    std::unique_ptr<dnn_core> core_;
  };

  template <Filter filtration_strategy>
  class trivial_cp_detector {
  public:
    std::unordered_map<size_t, check_point> extract_check_points(cv::Mat const& map, filtration_strategy filter = {}){
      return detector_.extract_point(filter.apply(map));
    }

  private:
    general_detector detector_;
  };
}


#endif //TRASH_ROGAINE_SOLVER_TRIVIAL_CP_DETECTOR_HPP
