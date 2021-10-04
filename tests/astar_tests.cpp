//
// Created by yalavrinenko on 26.09.2021.
//
#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "../src/pathfinders/legacy/asearch.hpp"
#include "../src/pathfinders/boost_graph/full_linked_graph.hpp"
#include <boost/format.hpp>
#include <iostream>

namespace std {
  ostream &operator<<(ostream &os, const pair<int, int> &yt) {
    os << (boost::format("(%d, %d)") % yt.first % yt.second);
    return os;
  }
}

BOOST_AUTO_TEST_SUITE(AstarTest)

BOOST_AUTO_TEST_CASE(SimpleGraphLoop, * boost::unit_test::tolerance(0.00001)){

  char data[] = {
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1,
      1, 1, 1, 1, 1
  };
  cv::Mat map {5, 5, CV_8UC1, data};

  trs::full_linked_grid graph{map};
  auto visitor = trs::full_linked_grid::visitor({4, 4});

  auto path = trs::astar_search<trs::full_linked_grid>::find_path(graph, {2, 2}, {4, 4}, visitor);
  BOOST_TEST(path.first == std::sqrt(8.0) );
  BOOST_TEST(path.second.size() == 3);

  std::vector<std::pair<int, int>> eta {{4, 4}, {3, 3}, {2, 2}};
  BOOST_TEST(path.second == eta, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(LinearGraph, * boost::unit_test::tolerance(0.00001)){

  char data[] = {
      1, 1, 1, 1, 1
  };
  cv::Mat map {1, 5, CV_8UC1, data};

  trs::full_linked_grid graph{map};
  trs::full_linked_grid::vertex_t begin{0, 0}, end{0, 4};
  auto visitor = trs::full_linked_grid::visitor(end);

  auto path = trs::astar_search<trs::full_linked_grid>::find_path(graph, begin, end, visitor);
  BOOST_TEST(path.first == 4 );
  BOOST_TEST(path.second.size() == 5);

  std::vector<std::pair<int, int>> eta {{0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0}};
  BOOST_TEST(path.second == eta, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(VerticalGraph, * boost::unit_test::tolerance(0.00001)){

  char data[] = {
      1,
      1,
      1,
      1,
      1
  };
  cv::Mat map {5, 1, CV_8UC1, data};

  trs::full_linked_grid graph{map};
  trs::full_linked_grid::vertex_t begin{4, 0}, end{0, 0};
  auto visitor = trs::full_linked_grid::visitor(end);

  auto path = trs::astar_search<trs::full_linked_grid>::find_path(graph, begin, end, visitor);
  BOOST_TEST(path.first == 4 );
  BOOST_TEST(path.second.size() == 5);

  std::vector<std::pair<int, int>> eta {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  BOOST_TEST(path.second == eta, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(NoPathGraph, * boost::unit_test::tolerance(0.00001)){

  char data[] = {
      1, 1, 1, 1,
      1, 1, 0, 0,
      1, 1, 0, 0,
  };
  cv::Mat map {3, 4, CV_8UC1, data};

  trs::full_linked_grid graph{map};
  trs::full_linked_grid::vertex_t begin{0, 0}, end{02, 3};
  auto visitor = trs::full_linked_grid::visitor(end);

  auto path = trs::astar_search<trs::full_linked_grid>::find_path(graph, begin, end, visitor);
  BOOST_TEST(path.first == -1.0 );
}

BOOST_AUTO_TEST_CASE(HardPathGraph, * boost::unit_test::tolerance(0.00001)){

  char data[] = {
      1, 0, 1, 1, 1,
      1, 1, 1, 1, 0,
      0, 0, 1, 0, 0,
      1, 1, 0, 0, 0,
      1, 1, 1, 1, 1
  };
  cv::Mat map {5, 5, CV_8UC1, data};

  trs::full_linked_grid graph{map};
  trs::full_linked_grid::vertex_t begin{0, 0}, end{4, 4};
  auto visitor = trs::full_linked_grid::visitor(end);

  auto path = trs::astar_search<trs::full_linked_grid>::find_path(graph, begin, end, visitor);
  auto s2 = std::sqrt(2.0);
  BOOST_TEST(path.first == (s2 + s2 + s2 + s2 + 1 + 1) );
  BOOST_TEST(path.second.size() == 7);

  std::vector<std::pair<int, int>> eta {
      {4, 4}, {4, 3}, {4, 2},
      {3, 1},
      {2, 2},
      {1, 1},
      {0, 0}
  };
  BOOST_TEST(path.second == eta, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_SUITE_END()

