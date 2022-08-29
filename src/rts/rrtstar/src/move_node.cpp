#include <ros/ros.h>
#include "rrtstar/move.h"

int main (int argc, char** argv){
    ros::init(argc, argv, "move_node");
    ros::NodeHandle nh;

    mover::Mover mover(nh);
    ros::spin();
    return 0;
}