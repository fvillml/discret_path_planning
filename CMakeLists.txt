cmake_minimum_required(VERSION 3.8)
project(BFS)
set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_SOURCE_DIR}/3rdParty)

ExternalProject_Add(matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    PREFIX matplotlib
    GIT_TAG master
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
)

add_library(path_planning SHARED
    src/Map.cpp 
    src/Planner.cpp
)
target_include_directories(path_planning PUBLIC include 3rdParty/include)
target_link_libraries(path_planning 
    Python3::Python
    Python3::Module
    Python3::NumPy
)

add_executable(bfs examples/bfs.cpp )
target_link_libraries(bfs path_planning)

add_executable(a_star examples/a_star.cpp )
target_link_libraries(a_star path_planning)

add_executable(from_map examples/from_map.cpp )
target_link_libraries(from_map path_planning)