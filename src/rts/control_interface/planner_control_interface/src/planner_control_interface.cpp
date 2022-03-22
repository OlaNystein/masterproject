
#include "planner_control_interface/planner_control_interface.h"

#include <chrono>
#include <thread>

namespace search {

PlannerControlInterface::PlannerControlInterface(
    ros::NodeHandle &nh, ros::NodeHandle &nh_private,
    std::shared_ptr<PCIManager> pci_manager)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_pub_ =
      nh_.advertise<planner_msgs::PlannerStatus>("gbplanner_status", 5);

  // Pose information.
  odometry_sub_ = nh_.subscribe(
      "odometry", 1, &PlannerControlInterface::odometryCallback, this);
  pose_sub_ =
      nh_.subscribe("pose", 1, &PlannerControlInterface::poseCallback, this);
  pose_stamped_sub_ = nh_.subscribe(
      "pose_stamped", 1, &PlannerControlInterface::poseStampedCallback, this);

  // Services and several standard services accompanied for easier trigger.
  pci_server_ =
      nh_.advertiseService("planner_control_interface_trigger",
                           &PlannerControlInterface::triggerCallback, this);
  
  pci_initialization_server_ = nh_.advertiseService(
      "pci_initialization_trigger",
      &PlannerControlInterface::initializationCallback, this);
  //



  

  
  pci_to_waypoint_server_ = nh_.advertiseService(
      "pci_to_waypoint", &PlannerControlInterface::goToWaypointCallback, this);


  
  pci_manager_ = pci_manager;
  if (!loadParams()) {
    ROS_ERROR("Can not load params. Shut down ROS node.");
    ros::shutdown();
  }

  if (!init()) {
    ROS_ERROR("Can not initialize the node. Shut down ROS node.");
    ros::shutdown();
  }
  run();
}




bool PlannerControlInterface::goToWaypointCallback(
    planner_msgs::pci_to_waypoint::Request &req,
    planner_msgs::pci_to_waypoint::Response &res) {
  go_to_waypoint_request_ = true;
  set_waypoint_.position.x = req.waypoint.position.x;
  set_waypoint_.position.y = req.waypoint.position.y;
  set_waypoint_.position.z = req.waypoint.position.z;
  set_waypoint_.orientation.x = req.waypoint.orientation.x;
  set_waypoint_.orientation.y = req.waypoint.orientation.y;
  set_waypoint_.orientation.z = req.waypoint.orientation.z;
  set_waypoint_.orientation.w = req.waypoint.orientation.w;
  return true;
}





bool PlannerControlInterface::initializationCallback(
    planner_msgs::pci_initialization::Request &req,
    planner_msgs::pci_initialization::Response &res) {
  init_request_ = true;
  res.success = true;
  return true;
}




bool PlannerControlInterface::triggerCallback(
    planner_msgs::pci_trigger::Request &req,
    planner_msgs::pci_trigger::Response &res) {
  if (pci_manager_->getStatus() == PCIManager::PCIStatus::kError) {
    ROS_WARN(
        "PCIManager is curretely in error state and cannot accept planning "
        "requests.");
    res.success = false;
  } else {
    if ((!req.set_auto) && (trigger_mode_ == PlannerTriggerModeType::kAuto)) {
      ROS_WARN("Switch to manual mode.");
      trigger_mode_ = PlannerTriggerModeType::kManual;
    } else if ((req.set_auto) &&
               (trigger_mode_ == PlannerTriggerModeType::kManual)) {
      ROS_WARN("Switch to auto mode.");
      trigger_mode_ = PlannerTriggerModeType::kAuto;
    }
    pci_manager_->setVelocity(req.vel_max);
    bound_mode_ = req.bound_mode;
    run_en_ = true;
    exe_path_en_ = !req.not_exe_path;
    res.success = true;
  }
  return true;
}


bool PlannerControlInterface::init() {
  planner_iteration_ = 0;

  run_en_ = false;
  exe_path_en_ = true;
  pose_is_ready_ = false;
  planner_msgs::planner_srv plan_srv_temp;
  bound_mode_ = plan_srv_temp.request.kExtendedBound;
  force_forward_ = true;
  init_request_ = false;
  global_request_ = false;

  go_to_waypoint_request_ = false;
  // Wait for the system is ready.
  // For example: checking odometry is ready.
  ros::Rate rr(1);
  bool cont = true;
  while (!pose_is_ready_) {
    ROS_WARN("Waiting for odometry.");
    ros::spinOnce();
    rr.sleep();
  }
  if (!pci_manager_->initialize()) return false;
  return true;
}

void PlannerControlInterface::run() {
  ros::Rate rr(10);  // 10Hz
  bool cont = true;
  while (cont) {
    PCIManager::PCIStatus pci_status = pci_manager_->getStatus();
    if (pci_status == PCIManager::PCIStatus::kReady) {
      // Priority 1: Check if initialize
      if (init_request_) {
        init_request_ = false;
        ROS_INFO("PlannerControlInterface: Running Initialization");
        runInitialization();
      }  // Priority 3: Stop.
      else if (go_to_waypoint_request_) {
        go_to_waypoint_request_ = false;
        pci_manager_->goToWaypoint(set_waypoint_);
      }
    } else if (pci_status == PCIManager::PCIStatus::kError) {
      // Reset everything to manual then wait for operator.
      resetPlanner();
    }
    cont = ros::ok();
    ros::spinOnce();
    rr.sleep();
  }
}





void PlannerControlInterface::publishPlannerStatus(
    const planner_msgs::planner_srv::Response &res, bool success) {
  planner_msgs::PlannerStatus planner_status_msg;
  planner_status_msg.header.stamp = ros::Time::now();
  planner_status_msg.header.frame_id = world_frame_id_;

  planner_status_msg.success = success;
  planner_status_msg.trigger_mode.mode =
      static_cast<decltype(planner_status_msg.trigger_mode.mode)>(
          trigger_mode_);
  planner_status_msg.bound_mode.mode == bound_mode_;
  planner_status_msg.max_vel = v_current_;

  // feeback from planner msg: N.A
  // planner_status_msg.planning_mode = res.planning_mode;
  // planner_status_msg.exe_path_mode = res.exe_path_mode;
  planner_status_pub_.publish(planner_status_msg);
}



void PlannerControlInterface::runInitialization() {
  ROS_WARN("Start initialization ...");
  pci_manager_->initMotion();
}

geometry_msgs::Pose PlannerControlInterface::getPoseToStart() {
  geometry_msgs::Pose ret;
  // use current state as default
  ret.position.x = 0.0;
  ret.position.y = 0.0;
  ret.position.z = 0.0;
  ret.orientation.x = 0.0;
  ret.orientation.y = 0.0;
  ret.orientation.z = 0.0;
  ret.orientation.w = 1.0;

  // Use the last waypoint as a starting pose if required to plan ahead
  if (pci_manager_->planAhead() && (current_path_.size()))
    ret = current_path_.back();
  return ret;
}

bool PlannerControlInterface::loadParams() {
  std::string ns = ros::this_node::getName();
  ROS_INFO("Loading: %s", ns.c_str());

  // Required params for robot interface.
  if (!pci_manager_->loadParams(ns)) return false;

  // Other params.
  std::string param_name;
  std::string parse_str;
  param_name = ns + "/trigger_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kManual"))
    trigger_mode_ = PlannerTriggerModeType::kManual;
  else if (!parse_str.compare("kAuto"))
    trigger_mode_ = PlannerTriggerModeType::kAuto;
  else {
    trigger_mode_ = PlannerTriggerModeType::kManual;
    ROS_WARN("No trigger mode setting, set it to kManual.");
  }

  param_name = ns + "/world_frame_id";
  if (!ros::param::get(param_name, world_frame_id_)) {
    world_frame_id_ = "world";
    ROS_WARN("No world_frame_id setting, set it to: %s.",
             world_frame_id_.c_str());
  }

  ROS_INFO("Done.");
  return true;
}

void PlannerControlInterface::odometryCallback(const nav_msgs::Odometry &odo) {
  current_pose_.position.x = odo.pose.pose.position.x;
  current_pose_.position.y = odo.pose.pose.position.y;
  current_pose_.position.z = odo.pose.pose.position.z;
  current_pose_.orientation.x = odo.pose.pose.orientation.x;
  current_pose_.orientation.y = odo.pose.pose.orientation.y;
  current_pose_.orientation.z = odo.pose.pose.orientation.z;
  current_pose_.orientation.w = odo.pose.pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  }
  pose_is_ready_ = true;
}

void PlannerControlInterface::resetPlanner() {
  // Set back to manual mode, and stop all current requests.
  trigger_mode_ = PlannerTriggerModeType::kManual;
  run_en_ = false;

  init_request_ = false;
  global_request_ = false;
  go_to_waypoint_request_ = false;

  // Remove the last waypoint to prevent the planner starts from that last wp.
  current_path_.clear();
  pci_manager_->setStatus(PCIManager::PCIStatus::kReady);
}

void PlannerControlInterface::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  processPose(pose.pose.pose);
}

void PlannerControlInterface::poseStampedCallback(
    const geometry_msgs::PoseStamped &pose) {
  processPose(pose.pose);
}

void PlannerControlInterface::processPose(const geometry_msgs::Pose &pose) {
  current_pose_.position.x = pose.position.x;
  current_pose_.position.y = pose.position.y;
  current_pose_.position.z = pose.position.z;
  current_pose_.orientation.x = pose.orientation.x;
  current_pose_.orientation.y = pose.orientation.y;
  current_pose_.orientation.z = pose.orientation.z;
  current_pose_.orientation.w = pose.orientation.w;
  pci_manager_->setState(current_pose_);
  if (!pose_is_ready_) {
    previous_pose_ = current_pose_;
  }
  pose_is_ready_ = true;
}
}  // namespace explorer

