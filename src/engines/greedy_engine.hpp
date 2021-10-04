//
// Created by yalavrinenko on 09.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_GREEDY_ENGINE_HPP
#define TRASH_ROGAINE_SOLVER_GREEDY_ENGINE_HPP
#include <utils/concepts.hpp>
#include <unordered_map>
#include <functional>

namespace trs{
  class greedy_engine{
  public:
    using distance_function = std::function<double(size_t, size_t)>;

    trs::solution solve(double target_distance, const std::unordered_map<size_t, check_point> &checks, size_t start,
                        distance_function distance) const;
  };

}
#endif//TRASH_ROGAINE_SOLVER_GREEDY_ENGINE_HPP
