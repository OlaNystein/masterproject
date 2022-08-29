cmake_minimum_required(VERSION 3.0.2)
project(rrtstar)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)



catkin_simple(ALL_DEPS_REQUIRED)



include_directories(
  ${Eigen_INCLUDE_DIRS}
)

cs_add_library(
  ${PROJECT_NAME}
  src/move.cpp
  src/rrtstar.cpp
  src/rrtstar_rviz.cpp
  src/rts.cpp
)





cs_add_executable(rts_node src/rts_node.cpp)
target_link_libraries(rts_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)





