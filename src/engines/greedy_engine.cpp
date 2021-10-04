//
// Created by yalavrinenko on 09.06.2021.
//
#include "greedy_engine.hpp"
#include <unordered_set>
#include <ranges>
#include <boost/format.hpp>

trs::solution trs::greedy_engine::solve(double target_distance, const std::unordered_map<size_t, check_point> &checks,
                                        size_t start, distance_function distance) const {
  solution sol{.distance = 0};
  sol.trace.emplace_back(start);

  std::unordered_set<size_t> visited{start};

  auto cutoff = 500.0;
  auto current = checks.at(start);

  auto nearest_filter = [&cutoff, &current](auto const& kv) {
    auto sqr = [](auto x) { return x * x; };
    return cutoff * cutoff > sqr(current.position.first - kv.second.position.first) + sqr(current.position.second - kv.second.position.second);
  };

  auto is_visited_filter = [&visited](auto const& kv) { return !visited.contains(kv.first); };

  auto start_distance_filter = [&distance, start, &sol, &target_distance](auto const &kv){
    return distance(kv.first, start) + sol.distance <= target_distance;
  };

  auto evaluate_distance = [&distance, &current, start](auto const& kv){
    auto d = distance(current.uid, kv.first);
    auto value = kv.first;
    while (value > 10)
      value /= 10;

    return std::pair{kv.first, static_cast<double>(value) / d};
  };

  do {
    auto nearest = checks | std::views::filter(is_visited_filter) | std::views::filter(nearest_filter) | std::views::filter(start_distance_filter);
    auto scores = nearest | std::views::transform(evaluate_distance) | std::views::common;

    if (!std::ranges::empty(scores)){
      auto max_score = std::ranges::max_element(scores, {}, &std::pair<size_t, double>::second);

      auto const &max_uid = (*max_score).first;
      visited.insert(max_uid);
      sol.trace.push_back(max_uid);
      sol.distance += distance(current.uid, max_uid);

      current = checks.at(max_uid);
      std::cout << (boost::format("\t\t*********Add check point %1% to solution distance left %2%") % current.uid %
                    (target_distance - sol.distance))
                << std::endl;
    } else {
      sol.trace.push_back(start);
      break;
    }
  } while (sol.distance < target_distance);

  return sol;
}
