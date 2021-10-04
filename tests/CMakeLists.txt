cmake_minimum_required(VERSION 3.18)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -fconcepts-diagnostics-depth=2")

find_package(OpenCV 4.5.2 REQUIRED)
find_package(Boost REQUIRED unit_test_framework log)

add_executable(astar_tests ../tests/astar_tests.cpp)
target_link_libraries(astar_tests Boost::unit_test_framework Boost::log ${OpenCV_LIBS} Threads::Threads)
target_include_directories(astar_tests PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../src"
        "${OpenCV_INCLUDE_DIRS}")