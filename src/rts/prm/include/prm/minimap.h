#ifndef MINIMAP_H_
#define MINIMAP_H_

#include <ros/ros.h>
#include "visualization_msgs/MarkerArray.h"
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/Image.h"
#include "planner_common/params.h"



namespace search {

class Minimap {
  public:
    Minimap(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    void setStatePtr(const std::vector<StateVec*>& states_for_minimap);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
   
    ros::Subscriber voxel_sub_;
    ros::Publisher minimap_pub_;

    sensor_msgs::Image minimap_;

    const std::vector<StateVec*>* states_for_minimap_;

    void voxelCallback(const visualization_msgs::MarkerArray& msg);

    static bool compareX(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    static bool compareY(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

    int fitXToYRange(double x, int XpointMax, int XpointMin);
    int fitYToXRange(double y, int YpointMax, int YpointMin);

    std::tuple<uint8_t, uint8_t, uint8_t> fitZtoRGB(double z);

    void colorUnit(int x, int y);

};



} // ns search

#endif