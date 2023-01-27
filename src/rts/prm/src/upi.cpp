#include "prm/upi.h"

namespace search{

Upi::planner_cli::planner_cli(Upi* upi,int id,const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
    upi_(upi), id_(id), nh_(nh), nh_private_(nh_private) {
  std::string uid = std::to_string(id);
  
  
  
  while(!(planner_client_ = nh_.serviceClient<rimapp_msgs::pci_plan_path_single>(
                                  "m100_" + uid + "/pci_plan_path_single", true))){
    ROS_WARN("UPI: waiting for PCI-service %d", id);
    sleep(1);
  }

  click_subscriber_ = nh_.subscribe("clicked_point_" + uid, 1, &Upi::planner_cli::clickCallback, this);

  ROS_INFO("UPI: connected to PCI %d", id);
}

Upi::Upi(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh), nh_private_(nh_private) {

  upi_planner_service_ = nh_.advertiseService("upi/single_planner", &Upi::upiPlanServiceCallback, this);
  upi_ma_planner_service_ = nh_.advertiseService("upi/ma_planner", &Upi::upiMAPlanServiceCallback, this);
  std::string ns = ros::this_node::getName();

  nh_.getParam(ns + "/num_robots", num_robots_);

  for (int i = 0; i < num_robots_; i++){
    planner_cli* p = new planner_cli(this, i, nh, nh_private);
    planner_clients_.push_back(p);
  }
  
}

void Upi::planner_cli::clickCallback(const geometry_msgs::PointStamped &p) {
  ROS_INFO("You clicked, %f, %f", p.point.x, p.point.y);

  upi_->callPciSimple(id_, p.point.x, p.point.y, p.point.z);
}

bool Upi::upiPlanServiceCallback(
  rimapp_msgs::upi_planner::Request& req,
  rimapp_msgs::upi_planner::Response& res){


  res.success = callPci(req.unit_id, req.target_pose);
  return true;
}

bool Upi::upiMAPlanServiceCallback(
  rimapp_msgs::upi_ma_planner::Request& req,
  rimapp_msgs::upi_ma_planner::Response& res){
  
  res.success = false;
  for (int i = 0; i < req.targets.size(); i++ ){
    res.success = callPciSimple(req.targets[i].unit_id, req.targets[i].x_target, req.targets[i].y_target, req.targets[i].y_target);
  }
  
  return res.success;
}


bool Upi::callPci(int id, geometry_msgs::Pose pose){
  rimapp_msgs::pci_plan_path_single upi_srv;
  //std::string service = planner_clients_[id]->planner_client_.getService();
  //ROS_INFO_STREAM("" << service);
  upi_srv.request.target = pose;
  upi_srv.request.unit_id = id;
  if(planner_clients_[id]->planner_client_.call(upi_srv)){
    ROS_INFO("UPI successfully called PCI for unit %d", id);
    return true;
  } else {
    ROS_WARN("UPI unable to communicate with PCI");
    return false;
  }
}

bool Upi::callPciSimple(int id, int x_target, int y_target, int z_target){
  rimapp_msgs::pci_plan_path_single upi_srv;
  geometry_msgs::Pose target_pose;
  target_pose.position.x = x_target;
  target_pose.position.y = y_target;
  target_pose.position.z = z_target + 2;
  target_pose.orientation.x = 0;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = 0;
  target_pose.orientation.w = 1;
  upi_srv.request.target = target_pose;
  upi_srv.request.unit_id = id;
  if(planner_clients_[id]->planner_client_.call(upi_srv)){
    ROS_INFO("UPI successfully called PCI");
    return true;
  } else {
    ROS_WARN("UPI unable to communicate with PCI");
    return false;
  }
}


} // ns search