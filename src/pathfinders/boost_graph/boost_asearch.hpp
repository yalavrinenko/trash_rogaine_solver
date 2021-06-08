//
// Created by yalavrinenko on 08.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP
#define TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <unordered_map>
#include <boost/unordered_set.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

namespace trs {
  class boost_grid_graph {
  public:
    using vertex_descriptor = boost::graph_traits<boost::grid_graph<2>>::vertex_descriptor;

    struct graph_vertex_hash {
      size_t operator()(vertex_descriptor const &u) const {
        size_t hash = 0;
        boost::hash_combine(hash, u[0]);
        boost::hash_combine(hash, u[1]);
        return hash;
      }
    };

    using hash = graph_vertex_hash;

    [[nodiscard]] vertex_descriptor vertex2d(size_t x, size_t y) const{
      return vertex(index_from_coord(x, y), graph_);
    }

    [[nodiscard]] auto const& graph() const {
      return filtered_graph_;
    }

  private:
    [[nodiscard]] auto index_from_coord(auto x, auto y) const -> size_t {
      return x * graph_.length(0) + y;
    }

    auto create_road_graph(cv::Mat const &map){
      for (std::weakly_incrementable auto col : std::views::iota(0, map.cols))
        for (std::weakly_incrementable auto row : std::views::iota(0, map.rows)) {
          if (map.at<char>(row, col) == 0) {
            auto iv = vertex(index_from_coord(row, col), graph_);
            ground_map_.insert(iv);
          }
        }
      return boost::make_vertex_subset_complement_filter(graph_, ground_map_);
    }

    boost::grid_graph<2ul> graph_;
    boost::unordered_set<vertex_descriptor, graph_vertex_hash> ground_map_;
    decltype(boost::make_vertex_subset_complement_filter(graph_, ground_map_)) filtered_graph_;

  public:
    explicit boost_grid_graph(cv::Mat const &map): graph_(
        boost::array<size_t, 2>{static_cast<unsigned long>(map.cols),
                                static_cast<unsigned long>(map.rows)}), filtered_graph_(create_road_graph(map)){
    }

    using graph_type = decltype(filtered_graph_);

    struct target_visitor : public boost::default_astar_visitor {
      struct target_reached {
      };

      explicit target_visitor(vertex_descriptor const &to) : target_{to} {}

      void examine_vertex(vertex_descriptor u, const decltype(filtered_graph_) &) {
        if (u == target_)
          throw target_visitor::target_reached();
      }

    private:
      vertex_descriptor target_;
    };
    using visitor_type = target_visitor;

    [[nodiscard]] visitor_type visitor(vertex_descriptor const& u) const{
      return target_visitor(u);
    }
  };

  //SEARCH
  template <typename Graph = boost_grid_graph::graph_type , typename Vertex = boost_grid_graph::vertex_descriptor ,
      typename Visitor = boost_grid_graph::visitor_type,
      typename Hash = boost_grid_graph::hash,
      typename StopException = boost_grid_graph::target_visitor::target_reached>
  class boost_astar_search {
  public:
    static std::pair<size_t, std::vector<std::pair<int, int>>> find_path(Graph const& g, const Vertex &s_vertex, const Vertex &g_vertex, Visitor v);
  private:
  };

  template<typename Graph, typename Vertex, typename Visitor, typename Hash, typename StopException>
  std::pair<size_t, std::vector<std::pair<int, int>>>
  boost_astar_search<Graph, Vertex, Visitor, Hash, StopException>::find_path(const Graph &g, const Vertex &s_vertex,
                                                              const Vertex &g_vertex, Visitor v) {
    std::unordered_map<Vertex, Vertex, Hash> predecessor;
    std::unordered_map<Vertex, double, Hash> distance;

    auto heuristic = [&g_vertex](Vertex const &u) {
      auto sqr = [](auto x) { return x * x; };
      return std::sqrt(sqr(g_vertex[0] - u[0]) + sqr(g_vertex[1] - u[1]));
    };

    try {
      boost::astar_search(g, s_vertex, heuristic,
                          boost::weight_map(boost::static_property_map<double>(1)).
                              predecessor_map(boost::associative_property_map<decltype(predecessor)>{predecessor}).
                              distance_map(boost::associative_property_map<decltype(distance)>{distance}).
                              visitor(v));
    }
    catch (StopException const &) {
      auto p = std::pair<size_t, std::vector<std::pair<int, int>>>{distance[g_vertex], {}};
      for (auto u = g_vertex; u != s_vertex; u = predecessor[u]) {
        p.second.emplace_back(u[0], u[1]);
      }
      p.second.emplace_back(s_vertex[0], s_vertex[1]);
      return p;
    }
    return {};
  }
}

#endif //TRASH_ROGAINE_SOLVER_BOOST_ASEARCH_HPP
