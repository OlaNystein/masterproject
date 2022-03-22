#ifndef RRTSTAR_H_
#define RRTSTAR_H_

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include "kdtree/kdtree.h"

#include "rrtstar/rrtstar_rviz.h"

#include "planner_common/graph_base.h"
#include "planner_common/graph.h"
#include "planner_common/graph_manager.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"
#include "planner_common/map_manager.h"
#include "planner_common/map_manager_voxblox_impl.h"
#include "planner_common/params.h"


namespace search{
namespace rrtstar{
    

class RobotStateHistory {
 public:
  RobotStateHistory();
  void addState(StateVec* s);
  bool getNearestState(const StateVec* state, StateVec** s_res);
  bool getNearestStateInRange(const StateVec* state, double range,
                              StateVec** s_res);
  bool getNearestStates(const StateVec* state, double range,
                        std::vector<StateVec*>* s_res);
  void reset();
  std::vector<StateVec*> state_hist_;

 private:
  // Kd-tree for nearest neigbor lookup.
  kdtree* kd_tree_;
};

class Rrtstar {
 public:
  enum TreeStatus {
    OK = 0,               // Everything good
    ERR_KDTREE,           // Could not get nearest neighbours from kdtree
    ERR_NO_FEASIBLE_PATH, // Could not find feasible path
    NOT_OK,               // Other error
  };

  Rrtstar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initialize graph to start a new planning session
  void reset();

  // Initialize some variables
  void initializeParams();

  // Load parameters from the rrtstar yaml config file
  bool loadParams();

  // Set the current robot's state (e.g. from odometry msg).
  void setState(StateVec& state);


  // Build a tree
  TreeStatus buildTree();

  ConnectStatus search(geometry_msgs::Pose source_pose,
                        geometry_msgs::Pose target_pose,
                        std::vector<geometry_msgs::Pose>& best_path);

  

 private:
  //------------VARS--------------

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Tree used for searching
  std::shared_ptr<GraphManager> tree_;
  // Shortest path report from last search
  ShortestPathsReport tree_rep_;
  // Vizualisation in rviz
  Visualization* visualization_;

  // Params required for planning.
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  PlanningParams planning_params_;
  RandomSamplingParams* random_sampling_params_;

  // Random samplers for planning/search
  RandomSampler random_sampler_;  
  BoundedSpaceParams global_space_params_;
  RobotDynamicsParams robot_dynamics_params_;


  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager_;
  
  // Received the robot's state from the localization module, ready to plan.
  bool odometry_ready;

  // Special vertices.
  Vertex* root_vertex_; //Root vertex for iterative planning
  Vertex* best_vertex_;

  // End of initial part of path.
  // When the robot is close to waypoint, planning 
  // for next segment stops and a new waypoint is calculated
  Vertex* current_waypoint_; 

  // Current state of the robot, updated from odometry.
  StateVec current_state_;
 

 

  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  // Statistic from the random sampling.
  std::shared_ptr<SampleStatistic> stat_;

  // To visualize the feasible corridors in the path refinement step.
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;



  //-----------FUNCTIONS-----------------

  bool sampleVertex(StateVec& state);

  void expandTree(std::shared_ptr<GraphManager> tree, StateVec& new_state, ExpandGraphReport& rep);//TODO

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


}
}

#endif