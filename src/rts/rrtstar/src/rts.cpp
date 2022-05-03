#include "rrtstar/rts.h"

#include <nav_msgs/Path.h>

namespace search {

RTS::RTS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private) {

  search_status_ = RTS::SearchStatus::NOT_READY;

  rrtstar_ = new rrtstar::Rrtstar(nh, nh_private);
  if (!(rrtstar_->loadParams())) {
    ROS_ERROR("Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  search_service_ = nh_.advertiseService("rts/search", &RTS::searchServiceCallback, this);
  build_tree_ = nh_.advertiseService("rts/buildtree", &RTS::buildTreeServiceCallback, this);

  pose_subscriber_ = nh_.subscribe("pose", 100, &RTS::poseCallback, this);
  pose_stamped_subscriber_ =
      nh_.subscribe("pose_stamped", 100, &RTS::poseStampedCallback, this);
  odometry_subscriber_ =
      nh_.subscribe("odometry", 100, &RTS::odometryCallback, this);
  
 

}

bool RTS::searchServiceCallback(rrtstar_msgs::search::Request& req,
    rrtstar_msgs::search::Response& res){
  ROS_INFO("Search service callback reached");
  res.path.clear();
  if (getSearchStatus() == RTS::SearchStatus::NOT_READY) {
    ROS_WARN("Search service not ready.");
    return false;
  }
  ROS_WARN("Printing target before searchrun x: %f, y: %f, z: %f. ", req.target.position.x, req.target.position.y, req.target.position.z);
  res.path = rrtstar_->runSearch(req.target);
  if (res.path.size() != 0 ) {
    ROS_WARN("Printing first pose of path after searchrun x: %f, y: %f, z: %f. ", res.path[0].position.x, res.path[0].position.y, res.path[0].position.z);
  } else {
    ROS_WARN("No best path returned");
  }
  res.final_target_reached = rrtstar_->getTargetStatus();
  return true;
}

bool RTS::buildTreeServiceCallback(
    rrtstar_msgs::build_tree::Request& req,
    rrtstar_msgs::build_tree::Response& res){
  res.success = false;
  if (!req.start){
    ROS_INFO("Build tree callback reached but not started");
    return res.success;
  }
  rrtstar_->reset();
  rrtstar::Rrtstar::TreeStatus treeStatus = rrtstar_->buildTree();
  if (treeStatus == rrtstar::Rrtstar::TreeStatus::OK){
    ROS_INFO("Tree Successfully built");
    res.success = true;
  } else {
    ROS_WARN("Could not build tree");
  }
  return res.success;
}

void RTS::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose) {
  processPose(pose.pose.pose);
}

void RTS::poseStampedCallback(const geometry_msgs::PoseStamped &pose) {
  processPose(pose.pose);
}

void RTS::processPose(const geometry_msgs::Pose &pose) {
  StateVec state;
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  state[3] = tf::getYaw(pose.orientation);
  rrtstar_->setState(state);
}

void RTS::odometryCallback(const nav_msgs::Odometry &odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  rrtstar_->setState(state);
}


RTS::SearchStatus RTS::getSearchStatus(){
  if (search_status_ == RTS::SearchStatus::READY){
    return RTS::SearchStatus::READY;
  }
  return RTS::SearchStatus::READY;
}


}