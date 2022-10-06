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

namespace search {

class RIMAPP {
    public:
    
    RIMAPP(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;


    prm::Prm* prm_;
};


} // namespace search

#endif