#include <ros/ros.h>

#include "rrtstar/rts.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rts_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    search::RTS rts(nh, nh_private);

    ros::spin();
    return 0;
}