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

struct Unit{
  int id_;
  Vertex* current_vertex_;
  StateVec current_state_;
  Vertex* current_waypoint_;
  StateVec final_target_;
  bool reached_final_target_;
}


class Prm {
  public:

  enum GraphStatus {
    OK = 0,               // Everything good
    ERR_KDTREE,           // Could not get nearest neighbours from kdtree
    ERR_NO_FEASIBLE_PATH, // Could not find feasible path
    NOT_OK,               // Other error
  };

  Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initialize parameters
  void initializeParams();

  //Load Parameters from the prm yaml config file
  bool loadParams();

  // Set the current robot's state (e.g. from odometry msg)
  void setState(StateVec& state, int unit_id);

  // Set the current active robot
  void setActiveUnit(int unit_id);

  // Plan a path from current position to target pose
  Prm::GraphStatus planPath(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path);

  std::vector<geometry_msgs::Pose> runPlanner(geometry_msgs::Pose& target_pose);

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

  bool lazy_mode_;

  // id of active robot to sample from 
  int active_id_;

  // List of robot position vertices
  std::vector<Vertex*> current_vertices_;
  // List of robot states
  std::vector<StateVec> current_states_;
  // List of current robot waypoints
  std::vector<Vertex*> current_waypoints_;

  std::vector<bool> final_targets_reached_;
  // Query queue
  // make a msg struct with query info
    // id, start, end
  
  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  //---------------------FUNCTIONS----------------------

  bool sampleVertex(StateVec& state);

  void expandGraph(std::shared_ptr<GraphManager> graph, StateVec& new_state, ExpandGraphReport& rep);

  //list of robots with id, lets start with 1 robot
  //one common roadmap
  //need to create a wrapper for a robot with the sampler and pathfinder
  // - How should sampling and adding to the same roadmap work? what is best computationally?
  //    - Should I test multiple solutions and have them in the results?
  // - Swarming/splitting

  void convertStateToPoseMsg(const StateVec& state, geometry_msgs::Pose& pose) {
    pose.position.x = state[0];
    pose.position.y = state[1];
    pose.position.z = state[2];
    double yawhalf = state[3] * 0.5;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sin(yawhalf);
    pose.orientation.w = cos(yawhalf);
  }

  void convertPoseMsgToState(const geometry_msgs::Pose& pose, StateVec& state) {
    state[0] = pose.position.x;
    state[1] = pose.position.y;
    state[2] = pose.position.z;
    state[3] = tf::getYaw(pose.orientation);
  }

  inline void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }

};

} //namespace search

} //namespace prm

#endif