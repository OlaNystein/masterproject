#ifndef PRM_H
#define PRM_H_

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>

#include "kdtree/kdtree.h"

#include "prm/prm_rviz.h"
#include "rrtstar_msgs/search.h"

#include "planner_common/graph_base.h"
#include "planner_common/graph.h"
#include "planner_common/graph_manager.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"
//#include "planner_common/map_manager.h"
#include "planner_common/map_manager_voxblox_impl.h"
#include "planner_common/params.h"

namespace search{
namespace prm{

class Prm {
  public:

  Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //list of robots with id, lets start with 1 robot
  //one common roadmap
  //need to create a wrapper for a robot with the sampler and pathfinder
  // - How should sampling and adding to the same roadmap work? what is best computationally?
  //    - Should I test multiple solutions and have them in the results?
  // - Swarming/splitting
}

} //namespace search

} //namespace prm

#endif