cmake_minimum_required(VERSION 3.10)
project(test_hybrid_astar)

include_directories(include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(osqp REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(test_hybrid_astar main.cpp src/astar_nodes/hastar_node.cpp
                                          src/astar_nodes/astar_node.cpp
                                          src/astar_nodes/node_utils.cpp
                                          src/hastar_optimizer.cpp
                                          src/astar_optimizer.cpp
                                          src/qp_optimizer.cpp
                                          src/solver_interface/osqp_interface.cpp
                                          src/solver_interface/csc_matrix_conv.cpp
                                          src/interpolate.cpp
                                          src/utils.cpp)

target_link_libraries(test_hybrid_astar PRIVATE osqp::osqp)

add_executable(filter_only main_filter_only.cpp src/astar_nodes/hastar_node.cpp
                                                src/astar_nodes/astar_node.cpp
                                                src/astar_nodes/node_utils.cpp
                                                src/hastar_optimizer.cpp
                                                src/astar_optimizer.cpp
                                                src/qp_optimizer.cpp
                                                src/solver_interface/osqp_interface.cpp
                                                src/solver_interface/csc_matrix_conv.cpp
                                                src/interpolate.cpp
                                                src/utils.cpp)

target_link_libraries(filter_only PRIVATE osqp::osqp)