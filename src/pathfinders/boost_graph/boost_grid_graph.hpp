//
// Created by yalavrinenko on 25.09.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_BOOST_GRID_GRAPH_HPP
#define TRASH_ROGAINE_SOLVER_BOOST_GRID_GRAPH_HPP

#endif//TRASH_ROGAINE_SOLVER_BOOST_GRID_GRAPH_HPP

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include "../graph_traits.hpp"

namespace trs{
  class boost_grid_graph {
  public:
    using vertex_t = boost::graph_traits<boost::grid_graph<2>>::vertex_descriptor;

    struct graph_vertex_hash {
      size_t operator()(vertex_t const &u) const {
        size_t hash = 0;
        boost::hash_combine(hash, u[0]);
        boost::hash_combine(hash, u[1]);
        return hash;
      }
    };

    using hash_t = graph_vertex_hash;

    [[nodiscard]] vertex_t vertex2d(size_t x, size_t y) const{
      return vertex(index_from_coord(x, y), graph_);
    }

    [[nodiscard]] auto const& graph() const {
      return filtered_graph_;
    }

  private:
    [[nodiscard]] auto index_from_coord(auto x, auto y) const -> size_t {
      return x * graph_.length(0) + y;
    }

    template<typename map_t>
    auto create_road_graph(map_t const &map){
      for (std::weakly_incrementable auto col : std::views::iota(0, map.cols))
        for (std::weakly_incrementable auto row : std::views::iota(0, map.rows)) {
          if (map.template at<char>(row, col) == 0) {
            auto iv = vertex(index_from_coord(row, col), graph_);
            ground_map_.insert(iv);
          }
        }
      return boost::make_vertex_subset_complement_filter(graph_, ground_map_);
    }

    boost::grid_graph<2ul> graph_;
    boost::unordered_set<vertex_t, graph_vertex_hash> ground_map_;
    decltype(boost::make_vertex_subset_complement_filter(graph_, ground_map_)) filtered_graph_;

  public:
    template<typename map_t>
    explicit boost_grid_graph(map_t const &map): graph_(
                                                        boost::array<size_t, 2>{static_cast<unsigned long>(map.cols),
                                                                                static_cast<unsigned long>(map.rows)}), filtered_graph_(create_road_graph(map)){
    }

    using graph_t = decltype(filtered_graph_);

    struct target_visitor : public boost::default_astar_visitor {
      struct target_reached {
      };

      explicit target_visitor(vertex_t const &to) : target_{to} {}

      void examine_vertex(vertex_t u, const decltype(filtered_graph_) &) {
        if (u == target_)
          throw target_visitor::target_reached();
      }

    private:
      vertex_t target_;
    };
    using visitor_t = target_visitor;

    [[nodiscard]] visitor_t visitor(vertex_t const& u) const{
      return target_visitor(u);
    }
  };

  template<>
  struct vertex_accessor<boost_grid_graph::vertex_t>{
    static size_t field(boost_grid_graph::vertex_t const& v, size_t id) {
      return v[id];
    }
  };
}