//
// Created by yalavrinenko on 08.06.2021.
//

#ifndef TRASH_ROGAINE_SOLVER_GRAPH_TRAITS_HPP
#define TRASH_ROGAINE_SOLVER_GRAPH_TRAITS_HPP

namespace trs {
  template<typename G>
  struct rogain_graph_traits {
    using graph_t = typename G::graph_t;
    using vertex_t = typename G::vertex_t;
    using visitor_t = typename G::visitor_t;
    using hash_t = typename G::hash_t;
    using stop_exception_t = typename G::visitor_t::target_reached; //fuck the boost graph library
  };


  template<typename vertex_type>
  struct vertex_accessor {
    static size_t field(vertex_type const& v, size_t id) { throw std::logic_error("Wrong vertex accessor"); }
  };
}

#endif//TRASH_ROGAINE_SOLVER_GRAPH_TRAITS_HPP
