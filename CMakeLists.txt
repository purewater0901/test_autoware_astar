cmake_minimum_required(VERSION 3.17)
project(test_hybrid_astar)

include_directories(include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 14)

add_executable(test_hybrid_astar main.cpp src/astar_nodes/hastar_node.cpp
                                          src/astar_nodes/node_utils.cpp
                                          src/astar_optimizer.cpp)