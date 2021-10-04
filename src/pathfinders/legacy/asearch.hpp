//
// Created by yalavrinenko on 25.09.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_ASEARCH_HPP
#define TRASH_ROGAINE_SOLVER_ASEARCH_HPP

#include "../graph_traits.hpp"
#include <queue>
#include <unordered_set>

namespace trs {

  //SEARCH
  template <typename Graph,
           typename Vertex = typename rogain_graph_traits<Graph>::vertex_t ,
           typename Visitor = typename rogain_graph_traits<Graph>::visitor_t,
           typename Hash = typename rogain_graph_traits<Graph>::hash_t,
           typename StopException = typename rogain_graph_traits<Graph>::stop_exception_t>
  class astar_search {
  public:
    static std::pair<double, std::vector<std::pair<int, int>>> find_path(Graph const&G, const Vertex &s_vertex, const Vertex &g_vertex, Visitor v);
  private:
  };


  template<typename Graph, typename Vertex, typename Visitor, typename Hash, typename StopException>
  std::pair<double, std::vector<std::pair<int, int>>>
  astar_search<Graph, Vertex, Visitor, Hash, StopException>::find_path(const Graph &G, const Vertex &s_vertex,
                                                                       const Vertex &g_vertex, Visitor v) {
    struct node {
      Vertex origin;
      double h, g, f;

      bool operator< (node const &rhs) const{
        return this->f > rhs.f;
      }
    };

    std::priority_queue<node> open_queue;
    std::unordered_set<Vertex, Hash> close_set;

    open_queue.push(node{s_vertex, 0., 0., 0.});

    std::unordered_map<Vertex, double, Hash> distance;
    distance[s_vertex] = 0.0;

    std::unordered_map<Vertex, Vertex, Hash> predecessor;

    auto edge_length = [](Vertex const& src, Vertex const& dst) -> double{
      auto sqr = [](auto x) -> double { return x * x; };
      return std::sqrt(sqr(vertex_accessor<Vertex>::field(src, 0) - vertex_accessor<Vertex>::field(dst, 0))
                       + sqr(vertex_accessor<Vertex>::field(src, 1) - vertex_accessor<Vertex>::field(dst, 1)));
    };

    auto heuristic = [&g_vertex, &edge_length](Vertex const &u) {
      return edge_length(g_vertex, u);
    };

    bool target_reached = false;
    while (!open_queue.empty()){
      auto vertex = open_queue.top(); open_queue.pop();

      close_set.insert(vertex.origin);
      auto successors_iterator = typename Graph::edge_iterator(vertex.origin, G);

      while (successors_iterator){
        auto [src, dst] = *successors_iterator;
        if (src == dst)
          break;

        try{
          v.examine_vertex(dst, G);
        } catch (StopException& e) {
          distance[dst] = vertex.g + edge_length(src, dst);
          predecessor[dst] = src;
          target_reached = true;
          break;
        }

        auto g = vertex.g + edge_length(src, dst);
        auto h = heuristic(dst);
        auto successor_node = node{dst, h, g, h + g};

        if (!distance.contains(dst) || distance[dst] > successor_node.g){
          open_queue.push(successor_node);
          distance[dst] = successor_node.g;
          predecessor[dst] = src;
        }

        successors_iterator++;
      };

      if (target_reached)
        break;
    }

    std::pair<double, std::vector<std::pair<int, int>>> path{-1.0, {}};
    if (target_reached) {
      path.first = distance[g_vertex];
      for (auto dst = g_vertex; dst != s_vertex; dst = predecessor[dst])
        path.second.emplace_back(std::make_pair<int, int>(vertex_accessor<Vertex>::field(dst, 0), vertex_accessor<Vertex>::field(dst, 1)));
      path.second.emplace_back(std::make_pair<int, int>(vertex_accessor<Vertex>::field(s_vertex, 0), vertex_accessor<Vertex>::field(s_vertex, 1)));
    }

    return path;
  }
}


#endif//TRASH_ROGAINE_SOLVER_ASEARCH_HPP
