#include <ros/ros.h>

#include <prm/rimapp.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rimapp_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //call rimapp node
  search::RIMAPP rimapp(nh, nh_private);

  ros::spin();
  return 0;
}