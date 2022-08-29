#include "rrtstar/move.h"

namespace mover {

Mover::Mover(const ros::NodeHandle& nh): nh_(nh)
{
    initVel();

    set_velocity_service_ = nh_.advertiseService("mover/set_vel", &Mover::setVelCallback, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("mover/velocity", 1000);

    velocity_sub_ = nh_.subscribe("mover/velocity", 1000, &Mover::velocityCallback, this);
}

void Mover::initVel(){
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
}

bool Mover::setVelCallback(rrtstar_msgs::velocity::Request &req, rrtstar_msgs::velocity::Response &res){
    
    ROS_INFO("Setting velocity");
    res.success = this->setVel(req);
    ROS_INFO("Successfull velocity change");


    return true;
}

bool Mover::setVel(rrtstar_msgs::velocity::Request req){
    this->vel = req.vel;

    velocity_pub_.publish(vel);
    return true;
}

void Mover::velocityCallback(const geometry_msgs::Twist& msg){
    ROS_INFO_STREAM(msg);
}

}