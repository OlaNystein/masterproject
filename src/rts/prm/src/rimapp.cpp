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
  
  plan_service_ = nh_.advertiseService("prm/plan", &RIMAPP::planServiceCallback, this);
  ROS_WARN("rimapp service advertised");
  // pose_subscriber_ = nh_.subscribe("pose", 100, &RIMAPP::poseCallback, this);
  // pose_stamped_subscriber_ =
  //     nh_.subscribe("pose_stamped", 100, &RIMAPP::poseStampedCallback, this);
  // odometry_subscriber_ =
  //     nh_.subscribe("odometry", 100, &RIMAPP::odometryCallback, this);

  }



bool RIMAPP::planServiceCallback(rimapp_msgs::plan_path_single::Request& req,
                           rimapp_msgs::plan_path_single::Response& res) {
  //
  res.stuck = false;
  //
  ROS_INFO("Single unit planner service reached");
  res.best_path.clear();
  ROS_WARN("Printing target before running planner x: %f, y: %f, z: %f. id: %d", req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z, req.unit_id);
  prm_->setActiveUnit(req.unit_id);

  res.best_path = prm_->runPlanner(req.target_pose);
 
  if (res.best_path.size() <= 1) {
    ROS_WARN("RIMAPP: No best path returned");
  }
  if (res.best_path.size() > 1){
    ROS_WARN("RIMAPP: Best path found");
  }
  res.final_target_reached = prm_->getTargetReachedSingle(req.unit_id);
  return true;
}

// void RIMAPP::runRimapp(){
//   ros::Rate rr(10);  // 10Hz
//   bool cont = true;
//   while(cont){
//     if (target_queue_.size() > 0){
//       geometry_msgs::Pose pose = target_queue_[0];
//       target_queue_.erase(target_queue_.begin());
//     }
//     cont = ros::ok();
//     ros::spinOnce();
//     rr.sleep();
//   }
// }
// pop, query, send path to pci, only erase if final target reached - start second query if there is one while moving

}// namespace search