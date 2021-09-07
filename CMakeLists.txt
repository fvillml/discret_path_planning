cmake_minimum_required(VERSION 3.8)
project(BFS)
set(CMAKE_CXX_STANDARD 17)

add_executable(bfs src/bfs.cpp )
target_include_directories(bfs PRIVATE include)

add_executable(a_star src/a_star.cpp )
target_include_directories(a_star PRIVATE include)