#include "prm/rimapp.h"

namespace search {


  RIMAPP::RIMAPP(onst ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private) {
  
  prm_ = new prm::Prm(nh, nh_private);

  if(!(prm_->loadParams())) {
    ROS_ERROR("Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  plan_service_ = nh_.advertiseService("prm/plan", &RIMAPP::planServiceCallback, this);

  pose_subscriber_ = nh_.subscribe("pose", 100, &RTS::poseCallback, this);
  pose_stamped_subscriber_ =
      nh_.subscribe("pose_stamped", 100, &RTS::poseStampedCallback, this);
  odometry_subscriber_ =
      nh_.subscribe("odometry", 100, &RTS::odometryCallback, this);

  }

  
}// namespace search