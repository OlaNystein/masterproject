#ifndef PLANNER_CONTROL_INTERFACE_H_
#define PLANNER_CONTROL_INTERFACE_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>

#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/BoundMode.h"
#include "planner_msgs/ExecutionPathMode.h"
#include "planner_msgs/PlannerStatus.h"
#include "planner_msgs/PlanningMode.h"
#include "planner_msgs/TriggerMode.h"
#include "planner_msgs/pci_geofence.h"
#include "planner_msgs/pci_global.h"
#include "planner_msgs/pci_homing_trigger.h"
#include "planner_msgs/pci_initialization.h"
#include "planner_msgs/pci_set_homing_pos.h"
#include "planner_msgs/pci_stop.h"
#include "planner_msgs/pci_to_waypoint.h"
#include "planner_msgs/pci_trigger.h"
#include "planner_msgs/planner_geofence.h"
#include "planner_msgs/planner_global.h"
#include "planner_msgs/planner_homing.h"
#include "planner_msgs/planner_request_path.h"
#include "planner_msgs/planner_set_exp_mode.h"
#include "planner_msgs/planner_set_homing_pos.h"
#include "planner_msgs/planner_srv.h"

#include "rrtstar_msgs/search.h"
#include "rrtstar_msgs/pci_search.h"
#include "rimapp_msgs/plan_path_single.h"
#include "rimapp_msgs/pci_plan_path_single.h"

namespace search {

// Macro for timing: notice that, have to use sequentially;
// otherwise, use different variables
typedef ros::Time TIMER;
#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

class PlannerControlInterface {
 public:
  enum struct RobotModeType { kAerialRobot = 0, kLeggedRobot };
  enum struct RunModeType {
    kSim = 0,  // Run in simulation.
    kReal,     // Run with real robot.
  };
  enum struct PlannerTriggerModeType {
    kManual = 0,  // Manually trigger the control interface each time.
    kAuto = 1     // Automatic exploration.
  };

  PlannerControlInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private,
                          std::shared_ptr<PCIManager> pci_manager);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

 private:
  ros::Publisher planner_status_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber pose_stamped_sub_;
  
  ros::ServiceClient rimapp_client_;
  ros::ServiceClient search_client_;
  ros::ServiceClient planner_client_;
  ros::ServiceClient planner_global_client_;
  ros::ServiceClient planner_geofence_client_;
  ros::ServiceClient planner_set_exp_mode_client_;
  
  ros::ServiceServer pci_surveillance_server_;
  ros::ServiceServer pci_rimapp_server_;
  ros::ServiceServer pci_search_server_;
  ros::ServiceServer pci_server_;
  ros::ServiceServer pci_std_automatic_planning_server_;
  ros::ServiceServer pci_initialization_server_;
  ros::ServiceServer pci_global_server_;
  ros::ServiceServer pci_std_global_server_;
  ros::ServiceServer pci_stop_server_;
  ros::ServiceServer pci_std_stop_server_;
  ros::ServiceServer pci_geofence_server_;
  ros::ServiceServer pci_to_waypoint_server_;
  ros::ServiceServer pci_std_set_vertical_exp_server_;
  ros::ServiceServer pci_std_set_horizontal_exp_server_;

  std::shared_ptr<PCIManager> pci_manager_;
  uint8_t bound_mode_;
  PlannerTriggerModeType trigger_mode_;
  double v_current_;
  bool run_en_;
  bool exe_path_en_;
  bool force_forward_;
  bool pose_is_ready_;
  bool init_request_;
  bool global_request_;

  geometry_msgs::Pose set_waypoint_;
  bool go_to_waypoint_request_;
  bool search_request_;

  planner_msgs::pci_global::Request pci_global_request_params_;
  int frontier_id_;

  // Current following path.
  std::vector<geometry_msgs::Pose> current_path_;

  int planner_iteration_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose previous_pose_;
  std::string world_frame_id_;

  void odometryCallback(const nav_msgs::Odometry &odo);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped &pose);
  void processPose(const geometry_msgs::Pose &pose);

  bool rimappCallback(rimapp_msgs::pci_plan_path_single::Request &req,
                      rimapp_msgs::pci_plan_path_single::Response &res);

  
  bool triggerCallback(planner_msgs::pci_trigger::Request &req,
                       planner_msgs::pci_trigger::Response &res);

  bool initializationCallback(planner_msgs::pci_initialization::Request &req,
                              planner_msgs::pci_initialization::Response &res);


  void publishPlannerStatus(const planner_msgs::planner_srv::Response &res, bool success);




  bool goToWaypointCallback(planner_msgs::pci_to_waypoint::Request &req,
                            planner_msgs::pci_to_waypoint::Response &res);

  bool rotate180DegCallback(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);

  
 

  void resetPlanner();

  bool loadParams();
  bool init();
  void run();

  void runInitialization();
  geometry_msgs::Pose getPoseToStart();

  bool use_current_state_;
  int search_iteration_;


  ros::Time ttime;
  double total_time_;
 

  //rimapp parameters
  bool rimapp_request_;
  bool stop_request_;
  geometry_msgs::Pose current_target_;
  bool target_reached_;
  int active_id_;
  void runRimapp();

};
}  // namespace explorer

#endif
