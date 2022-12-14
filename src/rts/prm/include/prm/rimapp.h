#ifndef RIMAPP_H_
#define RIMAPP_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "prm/prm_rviz.h"
#include "prm/prm.h"

#include "planner_common/graph.h"
#include "planner_common/params.h"

#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/planner_search.h"

#include "rimapp_msgs/plan_path_single.h"

#include <nav_msgs/Path.h>

#include <prm/upi.h>

namespace search {

class RIMAPP {
  public:
  
  RIMAPP(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);


  private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer plan_service_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;

  prm::Prm* prm_;

  Upi* upi_;


  bool planServiceCallback(
    rimapp_msgs::plan_path_single::Request& req,
    rimapp_msgs::plan_path_single::Response& res);

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
};


} // namespace search

#endif