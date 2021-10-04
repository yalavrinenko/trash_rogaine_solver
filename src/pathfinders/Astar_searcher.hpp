//
// Created by yalavrinenko on 01.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_ASTAR_SEARCHER_HPP
#define TRASH_ROGAINE_SOLVER_ASTAR_SEARCHER_HPP

#include "boost_graph/boost_asearch.hpp"
#include <utils/concepts.hpp>
#include "graph_traits.hpp"
#include <vector>

namespace trs {
  template<typename T>
  concept UndirectGraph = requires(T const &h) {
    {T(cv::Mat{})};
    {h.graph()};
    {h.visitor(std::declval<typename rogain_graph_traits<T>::vertex_t>())};
    {h.vertex2d(size_t{}, size_t{})};
  };

  template<typename T, typename graph>
  concept GraphSearchStrategy = requires(T s) {
    { s.find_path(std::declval<typename rogain_graph_traits<graph>::graph_t>(),
        std::declval<typename rogain_graph_traits<graph>::vertex_t>(),
        std::declval<typename rogain_graph_traits<graph>::vertex_t>(),
        std::declval<typename rogain_graph_traits<graph>::visitor_t>()) } -> std::convertible_to<path>;
  };

  template<RogaineMap map_t, UndirectGraph graph_handler = trs::boost_grid_graph,
           GraphSearchStrategy<graph_handler> search_strategy = trs::boost_astar_search<>>
  class Astar_searcher {
  public:
    explicit Astar_searcher(map_t const & map): handler_(map.extract_roads()) {}

    path find_path(check_point_vertex const &from, check_point_vertex const &to, search_strategy searcher = {}) {
      auto s_vertex = handler_.vertex2d(from.second, from.first);
      auto g_vertex = handler_.vertex2d(to.second, to.first);
      auto visitor = handler_.visitor(g_vertex);

      return searcher.find_path(handler_.graph(), s_vertex, g_vertex, visitor);
    }

    ~Astar_searcher() = default;

  private:
    graph_handler handler_;
  };
}// namespace trs

#endif//TRASH_ROGAINE_SOLVER_ASTAR_SEARCHER_HPP
