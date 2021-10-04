//
// Created by yalavrinenko on 22.05.2021.
//

#include <engines/greedy_engine.hpp>
#include <iostream>
#include <map/rogaine_map.hpp>
#include <pathfinders/Astar_searcher.hpp>
#include <solver.hpp>
#include <string>
#include <boost/format.hpp>

#include <pathfinders/boost_graph/full_linked_graph.hpp>
#include <pathfinders/boost_graph/boost_asearch.hpp>
#include <pathfinders/legacy/asearch.hpp>

int main(int argc, char** argv){
  //auto test_path = "../../demo/test.png";
  //auto test_path = "../../demo/16.jpg";
  auto test_path = "/home/yalavrinenko/Files/git/trash_rogaine_solver/demo/Serotonin Row 2021.png";

  trs::rogaine_map map(test_path);
  auto img = map.raw_image().clone();
  auto road = map.extract_roads();

  auto checks = map.checks();

  std::ranges::for_each(checks, [&img](auto const& check){
    auto position = cv::Point_<size_t>{check.second.position.first, check.second.position.second};
    cv::circle(img, position, 10, cv::Scalar{255}, 3);
    cv::putText(img, std::to_string(check.second.uid), position, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar{128}, 3);
  });

  using namespace std::string_literals;
  auto raw_window = test_path + " raw"s;

  auto scale = [](auto const &img, auto fx, auto fy) {
    cv::Mat out;
    cv::resize(img, out, {}, fx, fy);
    return out;
  };

  cv::imshow(raw_window, scale(img, 0.5, 0.5));
  //cv::imshow(raw_window, img);

//  cv::Mat hsv;
//  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
//
//  auto mouse_cb = [](int event, int x, int y, int flags, void* userdata){
//    auto* img = static_cast<cv::Mat*>(userdata);
//    if (event == cv::EVENT_LBUTTONDOWN){
//      std::cout << x << " " << y << " ===> ";
//      auto color = img->at<cv::Vec3b>(y, x);
//      std::cout << static_cast<int>(color[0]) << " " << static_cast<int>(color[1]) << " " << static_cast<int>(color[2]) << std::endl;
//    }
//  };
//
//  cv::setMouseCallback(raw_window, mouse_cb, &hsv);

//  cv::imshow(raw_window + "road", scale(road, 0.5, 0.5));
  cv::waitKey();

//  using searcher_t = trs::Astar_searcher < trs::rogaine_map<>, trs::full_linked_grid,
//        trs::boost_astar_search<trs::full_linked_grid, trs::full_linked_grid::vertex_t,
//                                trs::full_linked_grid::visitor_t, trs::full_linked_grid::hash_t,
//                                trs::full_linked_grid::target_visitor::target_reached>>;

//  using searcher_t = trs::Astar_searcher < trs::rogaine_map<>, trs::boost_grid_graph,
//                                         trs::boost_astar_search<> >;

  using searcher_t = trs::Astar_searcher < trs::rogaine_map<>, trs::full_linked_grid,
                                         trs::astar_search<trs::full_linked_grid> >;

  [[maybe_unused]] trs::solver<trs::rogaine_map<>, searcher_t, trs::greedy_engine> solver(map);

  double ED = 10000.0;
  size_t CP;

  std::cin >> CP >> ED;
  std::cout << "Solving for CP " << CP << " and ED " << ED << std::endl;

  auto sol = solver.solve(ED, CP);

  std::cout << boost::format("Solution for CP %1% and estimate distance %2%\n") % CP % ED;
  std::cout << boost::format("\tScore %1% \t Distance %2% \t Checks %3%\n") % sol.score % sol.distance % (sol.trace.size() - 1);

  std::cout << "Visited checks: ";
  std::ranges::copy(sol.trace | std::views::take(sol.trace.size() - 1), std::ostream_iterator<size_t>(std::cout, "->"));
  std::cout << sol.trace.back() << std::endl;

  for (auto &track : sol.track) {
    for (auto &tp : track.second) { cv::circle(img, cv::Point_<int>{tp.second, tp.first}, 2, cv::Scalar{0, 0, 255}, 2); }
  }

  cv::imshow(raw_window, scale(img, 0.6, 0.6));
  cv::waitKey();
  return 0;
}