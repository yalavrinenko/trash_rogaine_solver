//
// Created by yalavrinenko on 18.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_FULL_LINKED_GRAPH_HPP
#define TRASH_ROGAINE_SOLVER_FULL_LINKED_GRAPH_HPP

#include <array>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/functional/hash.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <utility>

namespace trs{
  class full_linked_grid {
  public:
    using vertex_t = std::array<size_t, 2>;
    using edge_t = std::pair<vertex_t, vertex_t>;

    struct target_visitor : public boost::default_astar_visitor {
      struct target_reached {
      };

      explicit target_visitor(vertex_t to) : target_{std::move(to)} {}

      void examine_vertex(vertex_t u, const full_linked_grid &) {
        if (u == target_)
          throw target_visitor::target_reached();
      }

    private:
      vertex_t target_;
    };
    using visitor_t = target_visitor;

    struct graph_vertex_hash {
      size_t operator()(vertex_t const &u) const {
        size_t hash = 0;
        boost::hash_combine(hash, u[0]);
        boost::hash_combine(hash, u[1]);
        return hash;
      }
    };
    using hash_t = graph_vertex_hash;

    explicit full_linked_grid(cv::Mat  mat): road_map_(std::move(mat)) {}

    [[nodiscard]] bool check_vertex(vertex_t const& v) const {
      return v[0] < static_cast<size_t>(road_map_.rows) && v[1] < static_cast<size_t>(road_map_.cols) &&
             road_map_.at<char>(static_cast<int>(v[0]), static_cast<int>(v[1]));
    }

    struct edge_iterator : public boost::iterator_facade<edge_iterator, edge_t, boost::forward_traversal_tag, edge_t>{
      edge_iterator(vertex_t vertex, full_linked_grid const &graph): graph_{graph}, current_(vertex){
        next_ = next_vertex();
      }

      edge_t operator* () const {
        return {current_, next_};
      }

      bool operator == (edge_iterator const& rhs) const {
        return current_ == rhs.current_ && next_ == rhs.next_;
      }

      bool operator != (edge_iterator const& rhs) const {
        return !(*this == rhs);
      }

      [[nodiscard]] bool equal(edge_iterator const& rhs) const {
        return this->operator==(rhs);
      }

      void operator++ () {
        next_ = next_vertex();
      }

      explicit operator bool() const {
        return has_vertex_;
      }

      void increment() {
        ++(*this);
      }

    private:
      vertex_t next_vertex() {
        has_vertex_ = false;
        next_ = current_;
        while (!has_vertex_ && current_shift_ < shifts_.size()){
          next_[0] = current_[0] + shifts_[current_shift_].first;
          next_[1] = current_[1] + shifts_[current_shift_].second;
          ++current_shift_;

          if (graph_.check_vertex(next_))
            has_vertex_ = true;
        }
        return next_;
      }

      size_t current_shift_ = 0;
      bool has_vertex_;
      full_linked_grid const &graph_;
      vertex_t current_;
      vertex_t next_;

      std::array<std::pair<int, int>, 8> shifts_ = {
          std::pair{-1, 1}, {0, 1}, {1, 1},
          {-1, 0}, {1, 0},
          {-1, -1}, {0, -1}, {1, -1}
      };
    };

    [[nodiscard]] auto& graph() const{
      return *this;
    }

    static visitor_t visitor(vertex_t const& v) {
      return target_visitor(v);
    }

    static vertex_t vertex2d(size_t row, size_t col) {
      return vertex_t {row, col};
    }

  private:
    cv::Mat road_map_;
  };

  template<>
  struct rogain_graph_traits<full_linked_grid>{
    using graph_t = full_linked_grid;
    using vertex_t = typename full_linked_grid::vertex_t;
    using visitor_t = typename full_linked_grid::visitor_t;
    using hash_t = typename full_linked_grid::hash_t;
    using stop_exception_t = full_linked_grid::visitor_t::target_reached;
  };

  template<>
  struct vertex_accessor<rogain_graph_traits<full_linked_grid>::vertex_t> {
    static size_t field(rogain_graph_traits<full_linked_grid>::vertex_t const& v, size_t id) {
      return v[id];
    }
  };
}

namespace boost{
  template<>
  struct graph_traits<trs::full_linked_grid>{
    using vertex_descriptor = trs::full_linked_grid::vertex_t;
    using edge_descriptor = trs::full_linked_grid::edge_t;
    using directed_category = boost::undirected_tag;
    using edge_parallel_category = boost::disallow_parallel_edge_tag;
    using traversal_category = boost::incidence_graph_tag;

    using out_edge_iterator = trs::full_linked_grid::edge_iterator;
    using degree_size_type = size_t;
    using edges_size_type = size_t;
  };
}

#endif//TRASH_ROGAINE_SOLVER_FULL_LINKED_GRAPH_HPP
