#ifndef PRM_UNIT_H_
#define PRM_UNIT_H_

namespace search{
namespace prm{

//wrapper for robot with sampler and pathfinding functions
class PrmUnit {
  public:

  PrmUnit(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  private:
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // - How should sampling and adding to the same roadmap work? what is best computationally?
  //    - Should I test multiple solutions and have them in the results?
  // - Swarming/splitting
}

} //namespace search

} //namespace prm

#endif