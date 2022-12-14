#ifndef UPI_H_
#define UPI_H


// load params num robots and create one client for each, and a server that redirects based on id
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include "rimapp_msgs/upi_planner.h"
#include "rimapp_msgs/pci_plan_path_single.h"
#include "geometry_msgs/Pose.h"

namespace search {

// User-Planner interface

class Upi {

  public:

  Upi(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  struct planner_cli {

    planner_cli(int id, const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceClient planner_client_;
    int id_;
  };

  private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Planner service exposed to user
  ros::ServiceServer upi_planner_service_;

  bool upiPlanServiceCallback(
    rimapp_msgs::upi_planner::Request& req,
    rimapp_msgs::upi_planner::Response& res);

  // Struct for dynamic creation of a client to each robot, 
  // subscribing to their pci-planner service


  std::vector<Upi::planner_cli*> planner_clients_;

  int num_robots_;

  bool single_plan_req_;

  void runService();

  bool callPci(int id, geometry_msgs::Pose pose);

  

};


} // ns search


#endif