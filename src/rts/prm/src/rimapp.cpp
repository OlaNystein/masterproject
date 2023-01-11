#include "prm/rimapp.h"

namespace search {


  RIMAPP::RIMAPP(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): 
    nh_(nh), nh_private_(nh_private) {
  prm_ = new prm::Prm(nh, nh_private);
  ROS_WARN("PRM CREATED");
  if(!(prm_->loadParams())) {
    ROS_ERROR("Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  upi_ = new Upi(nh, nh_private);

  best_path_pub_ = nh_.advertise<rimapp_msgs::Bestpath>("rimapp/best_path_res", 100);
  
  plan_service_ = nh_.advertiseService("rimapp/plan", &RIMAPP::planServiceCallback, this);
  ROS_WARN("rimapp service advertised");

  runRimapp();

  }



// bool RIMAPP::planServiceCallback(rimapp_msgs::plan_path_single::Request& req,
//                            rimapp_msgs::plan_path_single::Response& res) {
//   //
//   res.stuck = false;
//   //
//   ROS_INFO("Single unit planner service reached");
//   res.best_path.clear();
//   ROS_WARN("Printing target before running planner x: %f, y: %f, z: %f. id: %d", req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z, req.unit_id);
//   prm_->setActiveUnit(req.unit_id);

//   res.best_path = prm_->runPlanner(req.target_pose);
 
//   if (res.best_path.size() <= 1) {
//     ROS_WARN("RIMAPP: No best path returned");
//   }
//   if (res.best_path.size() > 1){
//     ROS_WARN("RIMAPP: Best path found");
//   }
//   res.final_target_reached = prm_->getTargetReachedSingle(req.unit_id);
//   return true;
// }

bool RIMAPP::planServiceCallback(rimapp_msgs::plan_path_single::Request& req,
                           rimapp_msgs::plan_path_single::Response& res) {
                          
  std::pair<geometry_msgs::Pose, int> p(req.target_pose, req.unit_id);
  target_queue_.push_back(p);
  ROS_INFO("RIMAPP: Order from unit %d added to queue", req.unit_id);
  res.success = true;
  return true;
}


void RIMAPP::runRimapp(){
  ros::Rate rr(10);  // 10Hz
  bool cont = true;
  while(cont){ 
    if (target_queue_.size() > 0){
      geometry_msgs::Pose target_pose = target_queue_[0].first;
      int id = target_queue_[0].second;
      ROS_INFO("RIMAPP:: planner for unit %d starting", id);
      target_queue_.erase(target_queue_.begin());
      ROS_INFO("Target queue size: %d", target_queue_.size());
      prm_->setActiveUnit(id);
      std::vector<geometry_msgs::Pose> best_path = prm_->runPlanner(target_pose);
      //publish results to pci-bestpath topic med ID, riktig pci kjører drone
      
      if (best_path.size() <= 1) {
        ROS_WARN("RIMAPP: No best path returned");
      }
      if (best_path.size() > 1){
        ROS_WARN("RIMAPP: Best path found");
      }

      rimapp_msgs::Bestpath res;
      res.unit_id = id;
      res.final_target_reached = prm_->getTargetReachedSingle(id);
      res.best_path = best_path;
      best_path_pub_.publish(res);
      ROS_INFO("RIMAPP:MSG published");
    }
    cont = ros::ok();
    ros::spinOnce();
    rr.sleep();
  }
}

}// namespace search