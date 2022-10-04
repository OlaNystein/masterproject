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

  std::shared_ptr<GraphManager> roadmap_graph_;

  ShortestPathsReport roadmap_graph_rep_;

  Visualization* visualization_;

  // Parameters required for planning
  SensorParams sensor_params_; // should be a list if different robots
  SensorParams free_frustum_params_;
  RobotParams robot_params_; //should be a list if different robots
  BoundedSpaceParams local_space_params_;
  PlanningParams planning_params_;
  RandomSamplingParams* random_sampling_params_;

  // Check which parameters are used
  RandomSampler random_sampler_;
  BoundedSpaceParams global_space_params_;
  RobotDynamicsParams robot_dynamics_params_;

  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager_;

  // Statistic from the random sampling.
  std::shared_ptr<SampleStatistic> stat_;

  // List of robots ready to update map
  std::vector<bool> odometry_ready_;

  // id of active robot to sample from 
  int* active_id_;

  // List of robot position vertices
  std::vector<Vertex*> root_vertices_;

  // Query queue
  // make a msg struct with query info
    // id, start, end

  //---------------------FUNCTIONS----------------------

  bool sampleVertex(StateVec& state);

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