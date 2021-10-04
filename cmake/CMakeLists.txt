cmake_minimum_required(VERSION 3.18)
project(trash_rogaine_solver)

option(USE_SANITIZER "Builde with asan support" OFF)

find_package(OpenCV 4.5.2 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED log)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -fconcepts-diagnostics-depth=2")

set(main_src ../src/main.cpp
        ../src/map/rogaine_map.cpp ../src/map/rogaine_map.hpp
        ../src/map/ocv_image_loader.cpp ../src/map/ocv_image_loader.hpp
        ../src/utils/exceptions.hpp ../src/utils/concepts.hpp
        ../src/map/filters/water_filter.cpp ../src/map/filters/water_filter.hpp
        ../src/map/filters/target_filter.cpp ../src/map/filters/target_filter.cpp
        ../src/map/filters/target_filter.hpp ../src/map/detectors/trivial_cp_detector.cpp
        ../src/map/detectors/trivial_cp_detector.hpp ../src/pathfinders/Astar_searcher.cpp ../src/pathfinders/Astar_searcher.hpp ../src/solver.cpp ../src/solver.hpp ../src/pathfinders/boost_graph/boost_asearch.hpp ../src/pathfinders/graph_traits.hpp ../src/engines/greedy_engine.hpp ../src/engines/greedy_engine.cpp ../src/pathfinders/boost_graph/full_linked_graph.hpp ../src/pathfinders/boost_graph/boost_grid_graph.hpp ../src/pathfinders/legacy/asearch.hpp)

set(libs ${OpenCV_LIBS} Threads::Threads Boost::log)

set(main_target_name TRSolver)
add_executable(${main_target_name} ${main_src})

target_include_directories(${main_target_name} PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../src"
        "${OpenCV_INCLUDE_DIRS}")

if (USE_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g")
    target_link_libraries(${main_target_name} ${libs})
else()
    target_link_libraries(${main_target_name} ${libs})
endif()

add_subdirectory(../tests ${CMAKE_CURRENT_BINARY_DIR}/tests)