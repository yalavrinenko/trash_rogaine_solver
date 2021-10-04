//
// Created by yalavrinenko on 08.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP
#define TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP

#include <boost/graph/astar_search.hpp>
#include "boost_grid_graph.hpp"
#include <unordered_map>
#include <boost/unordered_set.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

#include <boost/graph/vertex_and_edge_range.hpp>

namespace trs {

  //SEARCH
  template <typename Graph = boost_grid_graph::graph_t, typename Vertex = boost_grid_graph::vertex_t,
      typename Visitor = boost_grid_graph::visitor_t,
      typename Hash = boost_grid_graph::hash_t,
      typename StopException = boost_grid_graph::target_visitor::target_reached>
  class boost_astar_search {
  public:
    static std::pair<double, std::vector<std::pair<int, int>>> find_path(Graph const& g, const Vertex &s_vertex, const Vertex &g_vertex, Visitor v);
  private:
  };

  template<typename Graph, typename Vertex, typename Visitor, typename Hash, typename StopException>
  std::pair<double, std::vector<std::pair<int, int>>>
  boost_astar_search<Graph, Vertex, Visitor, Hash, StopException>::find_path(const Graph &g, const Vertex &s_vertex,
                                                              const Vertex &g_vertex, Visitor v) {
    std::unordered_map<Vertex, Vertex, Hash> predecessor;
    std::unordered_map<Vertex, double, Hash> distance;

    auto heuristic = [&g_vertex](Vertex const &u) {
      auto sqr = [](auto x) { return x * x; };
      return std::sqrt(sqr(vertex_accessor<Vertex>::field(g_vertex, 0) - vertex_accessor<Vertex>::field(u, 0))
                       + sqr(vertex_accessor<Vertex>::field(g_vertex, 1) - vertex_accessor<Vertex>::field(u, 1)));
    };

    try {
      boost::astar_search(g, s_vertex, heuristic,
                          boost::weight_map(boost::static_property_map<double>(1)).
                              predecessor_map(boost::associative_property_map<decltype(predecessor)>{predecessor}).
                              distance_map(boost::associative_property_map<decltype(distance)>{distance}).
                              visitor(v));
    }
    catch (StopException const &) {
      auto p = std::pair<double, std::vector<std::pair<int, int>>>{distance[g_vertex], {}};
      for (auto u = g_vertex; u != s_vertex; u = predecessor[u]) {
        p.second.emplace_back(vertex_accessor<Vertex>::field(u, 0), vertex_accessor<Vertex>::field(u, 1));
      }
      p.second.emplace_back(vertex_accessor<Vertex>::field(s_vertex, 0), vertex_accessor<Vertex>::field(s_vertex, 1));
      return p;
    }
    return {};
  }
}

#endif //TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP
