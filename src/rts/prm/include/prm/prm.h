#ifndef PRM_H_
#define PRM_H_

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include "kdtree/kdtree.h"

#include "prm/prm_rviz.h"
#include <prm/minimap.h>

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

  enum GraphStatus {
    OK = 0,               // Everything good
    ERR_KDTREE,           // Could not get nearest neighbours from kdtree
    ERR_NO_FEASIBLE_PATH, // Could not find feasible path
    NOT_OK,               // Other error
  };

  enum StateStatus {
    UNINITIALIZED = 0,    // Unit is uninitialized
    EMPTYGRAPH,           // The graph is empty
    CONNECTED,            // Unit is near vertex
    DISCONNECTED,         // Unit is completely disconnected from graph
    ERROR,                // Could not add state to graph
  };

  struct unit{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    unit(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    int id_;
    Vertex* current_vertex_;
    StateVec current_state_;
    Vertex* current_waypoint_;
    StateVec final_target_;
    bool reached_final_target_;
    Prm::StateStatus unit_status_;
    Prm::StateStatus target_status_;
    double pcl_clear_radius_;

    std::vector<int> current_path_id_list_;
    bool currently_moving_;
    ros::Time moving_time_start_;

    int publish_throttle_it_;

    const std::vector<Prm::unit*>* units_for_pcl_;
    void setUnitPtr(const std::vector<Prm::unit*>& units_for_pcl);

    void odometryCallback(const nav_msgs::Odometry& odo){
      StateVec state;
      state[0] = odo.pose.pose.position.x;
      state[1] = odo.pose.pose.position.y;
      state[2] = odo.pose.pose.position.z;
      state[3] = tf::getYaw(odo.pose.pose.orientation);
      // Directly updates the current state
      current_state_ = state;
    }

    tf::TransformListener tf_listener_;

    ros::Subscriber odometry_sub_;
    ros::Subscriber pointcloud_sub_;

    ros::Publisher pointcloud_pub_;

    void pclCallback(const sensor_msgs::PointCloud2& pcl);
    void setOdomSubscriber(std::string odom_prefix);
    void setPclSubscriber(std::string pcl_prefix);
  

    void setID(int id){
      id_ = id;
    }

    void setClearRad(double clear_rad){
      pcl_clear_radius_ = clear_rad;
    }

  };

  Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initialize parameters
  void initializeParams();

  //Load Parameters from the prm yaml config file
  bool loadParams();

  // Set the current active robot
  void setActiveUnit(int unit_id);

  bool getTargetReachedSingle(int unit_id);

  int getActiveUnit();

  void hardReset(); // Reset graph
  void softReset(); //Reinitialize planner but keep graph

  // Plan a path from current position to target pose
  Prm::GraphStatus planPath(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path);

  std::vector<geometry_msgs::Pose> runPlanner(geometry_msgs::Pose& target_pose);

  void expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                        Vertex* new_vertex, ExpandGraphReport& rep);
  
  bool connectStateToGraph(std::shared_ptr<GraphManager> graph,
                              StateVec& cur_state, Vertex*& v_added,
                              double dist_ignore_collision_check);

  int getNumRobots(){
    return num_robots_;
  }

  void setUnitMovingState(int id, bool moving){
    if (id < getNumRobots()){
      units_[id]->currently_moving_ = moving;
      if (!moving){
        units_[id]->current_path_id_list_.clear();
      } else {
        units_[id]->moving_time_start_ = ros::Time::now();
      }
    }
    return;
  }


  private:
  
  

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::shared_ptr<GraphManager> roadmap_graph_;

  ShortestPathsReport roadmap_graph_rep_;

  std::vector<Visualization*> visualization_;

  Minimap* minimap_;

  // Parameters required for planning
  SensorParams sensor_params_; 
  SensorParams free_frustum_params_;
  RobotParams robot_params_; 
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

  int num_robots_;
  // Query queue

  std::vector<Prm::unit*> units_;
  std::vector<std::pair<int, StateVec*>> cur_states_;
  // make a msg struct with query info
    // id, start, end
  
  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  //---------------------FUNCTIONS----------------------
  void detectUnitStatus(int unit_id);

  void detectTargetStatus(int unit_id);

  void addStartVertex();

  bool sampleVertex(StateVec& state);

  void expandGraph(std::shared_ptr<GraphManager> graph, StateVec& new_state, ExpandGraphReport& rep);

  bool checkCollisionBetweenVertices(Vertex* v_start, Vertex* v_end);

  bool doCuboidsIntersect(const Eigen::AlignedBox3d &cuboid1, const Eigen::AlignedBox3d &cuboid2);

  void printUnit(int unit_id);

  // bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
  //                           const std::vector<geometry_msgs::Pose>& path);
  // bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
  //                    Eigen::Vector3d& p0, Eigen::Vector3d& p1,
  //                    Eigen::Vector3d& p1_mod);
  // bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
  //                         std::vector<geometry_msgs::Pose>& path_mod);
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