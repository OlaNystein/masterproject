#ifndef MOVE_H_
#define MOVE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <rrtstar_msgs/velocity.h>

namespace mover {

class Mover {
    public:

    Mover(const ros::NodeHandle& nh);

    
    geometry_msgs::Twist getVel();

    private:

    void initVel();
    ros::NodeHandle nh_;
    ros::ServiceServer set_velocity_service_;
    ros::Publisher velocity_pub_;
    ros::Subscriber velocity_sub_;
    geometry_msgs::Twist vel;

    bool setVelCallback(rrtstar_msgs::velocity::Request &req, rrtstar_msgs::velocity::Response &res);
    bool setVel(rrtstar_msgs::velocity::Request req);
    void velocityCallback(const geometry_msgs::Twist& msg);
    
};

}

#endif
