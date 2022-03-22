cmake_minimum_required(VERSION 3.0.2)
project(planner_common)

add_definitions(-std=c++14 -Werror=return-type -Wno-ignored-attributes -Wno-int-in-bool-context)

# Point collision checking:
#   0 - Orignal
#   1 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
option(COL_CHECK_METHOD "Select collision checking method for sampled point")
add_definitions(-DCOL_CHECK_METHOD=${COL_CHECK_METHOD})
MESSAGE("COL_CHECK_METHOD? " ${COL_CHECK_METHOD})
# Line collision checking: 
#   0 - Orignal
#   1 - Cuboid around the edge
#   2 - Direct TSDF/ESDF distance check (requires continuous path check during execution)
option(EDGE_CHECK_METHOD "Select collision checking method for an edge")
set(EDGE_CHECK_METHOD 1)
add_definitions(-DEDGE_CHECK_METHOD=${EDGE_CHECK_METHOD})
MESSAGE("EDGE_CHECK_METHOD? " ${EDGE_CHECK_METHOD})
# Ray casting: 
#   0 - Original
#   1 - Iterative ray casting
option(RAY_CAST_METHOD "Select ray casting method")
set(RAY_CAST_METHOD 1)
add_definitions(-DRAY_CAST_METHOD=${RAY_CAST_METHOD})
MESSAGE("RAY_CAST_METHOD? " ${RAY_CAST_METHOD})

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp_serialization REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/octomap/map_manager_octomap_impl.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
  src/geofence_manager.cpp
)

cs_install()
cs_export(LIBRARIES)