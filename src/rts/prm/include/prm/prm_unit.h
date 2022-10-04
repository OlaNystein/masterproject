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

/*
Unit function:
ExpandLocalGraph (PRM) (sample one point at a time, or more? or should)
UpdateGlobalGraph (PRM)
QueryGlobalGraph/GetShortestPath (ShortestPath) - collision check done here if lazy PRM
    Corridor around returned path is treated as obstructed (reserved) while path segment is executed. If short enough segments,
    time could maybe be neglected

Central function:
UpdateGlobalGraph (PRM) (should be somewhat trivial if each unit only
                         pushes one point at a time to a queue, 
                         harder if units tries to update with multiple)
PublishGlobalGraph (PRM) - broadcast to all single units and team-leaders
FindSinglePath (shortest path with collisionchecking)
FindMultiPath (how to scale this most efficiently to avoid collisions between units?)

*/