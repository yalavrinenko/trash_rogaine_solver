//
// Created by yalavrinenko on 07.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_SOLVER_HPP
#define TRASH_ROGAINE_SOLVER_SOLVER_HPP

#include <utils/concepts.hpp>
#include <map>

namespace trs {
  template<typename T, typename M>
  concept PathSearcher = requires(T const &t) {
    {T{std::declval<M>()}};
  };

  template<typename T>
  concept SolverEngine = requires(T const &s) {
    { s.solve(0.0, std::unordered_map<size_t, check_point>{}, size_t{},
              std::function<double(size_t, size_t)>{}) } -> std::convertible_to<solution>;
  };

  template<RogaineMap map_t,
      PathSearcher<map_t> search_strategy,
      SolverEngine solving_strategy>
  class solver {
  public:
    explicit solver(map_t &map): check_points_{map.checks()} {
      auto filter = [this](cv::Mat &image) {
        std::ranges::for_each(check_points_, [&image](auto const &kv) {
          auto &position = kv.second.position;
          cv::circle(image, cv::Point(position.first, position.second), 10, cv::Scalar{255}, 1, cv::FILLED);
        });
      };

      map.filter_terrain(filter);
      searcher_ = std::make_unique<search_strategy>(map);
    }

    solution solve(double target_distance, size_t start, solving_strategy solver_engine = {}) {
      auto distance_function = [this](size_t from, size_t to) -> double {
        auto pair = std::make_pair(from, to);
        if (!distance_map_.count(pair)){
          auto epath = searcher_->find_path(check_points_[from].position, check_points_[to].position);
          if (epath.second.empty())
            epath.first = -1.0;
          distance_map_[pair] = epath;
          distance_map_[{to, from}] = distance_map_[pair];
        }
        return distance_map_[pair].first;
      };

      auto raw_solution = solver_engine.solve(target_distance, check_points_, start, distance_function);
      for (auto current = start; auto &check : raw_solution.trace | std::views::drop(1)){
          raw_solution.score += check / 10;
          raw_solution.track.emplace_back(distance_map_[{current, check}]);
          current = check;
      }

      return raw_solution;
    }

    void reset() {
      distance_map_.clear();
    }

  private:
    std::unique_ptr<search_strategy> searcher_ = nullptr;
    std::unordered_map<size_t, check_point> check_points_;

    std::map<check_point_vertex, path> distance_map_;
  };
}// namespace trs


#endif//TRASH_ROGAINE_SOLVER_SOLVER_HPP
