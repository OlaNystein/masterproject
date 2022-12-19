#include "prm/upi.h"

namespace search{

Upi::planner_cli::planner_cli(int id,const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
    id_(id), nh_(nh), nh_private_(nh_private) {
  std::string uid = std::to_string(id);

  
  
  while(!(planner_client_ = nh_.serviceClient<rimapp_msgs::pci_plan_path_single>(
                                  "m100_" + uid + "/pci_plan_path_single", true))){
    ROS_WARN("UPI: waiting for PCI-service %d", id);
    sleep(1);
  }
  ROS_INFO("UPI: connected to PCI %d", id);
}

Upi::Upi(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh), nh_private_(nh_private) {

  upi_planner_service_ = nh_.advertiseService("upi/single_planner", &Upi::upiPlanServiceCallback, this);
  
  std::string ns = ros::this_node::getName();

  nh_.getParam(ns + "/num_robots", num_robots_);

  for (int i = 0; i < num_robots_; i++){
    planner_cli* p = new planner_cli(i, nh, nh_private);
    planner_clients_.push_back(p);
  }

  //single_plan_req_ = false;

  
}

bool Upi::upiPlanServiceCallback(
  rimapp_msgs::upi_planner::Request& req,
  rimapp_msgs::upi_planner::Response& res){

  //single_plan_req_ = true;
  res.success = callPci(req.unit_id, req.target_pose);
  return true;
}

// void upi::runService(){
//   while(true){
//     if(single_plan_req_){
//       callPci();
//     }
//   }
// }

bool Upi::callPci(int id, geometry_msgs::Pose pose){
  rimapp_msgs::pci_plan_path_single upi_srv;
  std::string service = planner_clients_[id]->planner_client_.getService();
  //ROS_INFO_STREAM("" << service);
  upi_srv.request.target = pose;
  upi_srv.request.unit_id = id;
  if(planner_clients_[id]->planner_client_.call(upi_srv)){
    ROS_INFO("UPI successfully called PCI");
    //single_plan_req_ = false;
    return true;
  } else {
    ROS_WARN("UPI unable to communicate with PCI");
    return false;
  }
}


} // ns search