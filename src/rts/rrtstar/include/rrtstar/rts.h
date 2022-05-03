#ifndef RTS_H_
#define RTS_H_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "rrtstar/rrtstar_rviz.h"
#include "rrtstar/rrtstar.h"

#include "planner_common/graph.h"
#include "planner_common/params.h"

#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/planner_search.h"

#include "rrtstar_msgs/build_tree.h"
#include "rrtstar_msgs/search.h"


namespace search {

class RTS {
 public:
  enum SearchStatus {NOT_READY = 0, READY};
  enum SearchState {kNull = 0, kStart, kBoxClearance, kSearching, kStop};
  RTS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer search_service_;
  ros::ServiceServer build_tree_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;

  SearchStatus search_status_;
  SearchState search_state_;

  rrtstar::Rrtstar* rrtstar_;

  bool searchServiceCallback(
    rrtstar_msgs::search::Request& req,
    rrtstar_msgs::search::Response& res);
  
  bool buildTreeServiceCallback(
    rrtstar_msgs::build_tree::Request& req,
    rrtstar_msgs::build_tree::Response& res);


  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);

  SearchStatus getSearchStatus();
};



}

#endif