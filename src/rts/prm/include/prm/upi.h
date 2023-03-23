#ifndef UPI_H_
#define UPI_H


// load params num robots and create one client for each, and a server that redirects based on id
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include "rimapp_msgs/upi_planner.h"
#include "rimapp_msgs/upi_ma_planner.h"
#include "rimapp_msgs/pci_plan_path_single.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"

namespace search {

// User-Planner interface

class Upi {

  public:

  Upi(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  struct planner_cli {

    planner_cli(Upi* upi, int id, const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceClient planner_client_;
    int id_;

    Upi* upi_;


    ros::Subscriber click_subscriber_;
    void clickCallback(const geometry_msgs::PointStamped &pt);
  };

  private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Planner service exposed to user
  ros::ServiceServer upi_planner_service_;
  ros::ServiceServer upi_ma_planner_service_;

  bool upiPlanServiceCallback(
    rimapp_msgs::upi_planner::Request& req,
    rimapp_msgs::upi_planner::Response& res);

  // Struct for dynamic creation of a client to each robot, 
  // subscribing to their pci-planner service
  bool upiMAPlanServiceCallback(
  rimapp_msgs::upi_ma_planner::Request& req,
  rimapp_msgs::upi_ma_planner::Response& res);


  std::vector<Upi::planner_cli*> planner_clients_;

  int num_robots_;

  bool single_plan_req_;


  bool callPci(int id, geometry_msgs::Pose pose);

  bool callPciSimple(int id, int x_target, int y_target, int z_target);

};


} // ns search


#endif