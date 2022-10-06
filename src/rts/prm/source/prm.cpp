#include "prm/prm.h"

namespace search{
namespace prm{


Prm::Prm(onst ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){

    visualization_ = new Visualization(nh_, nh_private_);

    roadmap_graph_.reset(new GraphManager());

    map_manager_->resetMap();

    //TODO initialize odometry_ready to size of robotvector

    for (int i = 0; i < odometry_ready_.size()){
        odometry_ready_[i] = false;
    }





}

void Prm::setState(StateVec& state, int unit_id){
  if (!odometry_ready) {
    // First time receive the pose/odometry for planning purpose.
    // Reset the voxblox map
    ROS_WARN("Received the first odometry from unit %d", unit_id);
  }
  current_states_[unit_id] = state;
  odometry_ready = true;
}

bool Prm::sampleVertex(StateVec& state) {
  bool found = false;
  int while_thresh = 1000; // tuning param

  while (!found && while_thresh--){
      
    random_sampler_.generate(current_vertices[*active_id_]->state, state);

    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    if (state.x() + robot_params_.center_offset.x() <
        global_space_params_.min_val.x() + 0.5 * robot_box_size_.x()) {
    continue;
    } else if (state.y() + robot_params_.center_offset.y() <
            global_space_params_.min_val.y() + 0.5 * robot_box_size_.y()) {
    continue;
    } else if (state.z() + robot_params_.center_offset.z() <
            global_space_params_.min_val.z() + 0.5 * robot_box_size_.z()) {
    continue;
    } else if (state.x() + robot_params_.center_offset.x() >
            global_space_params_.max_val.x() - 0.5 * robot_box_size_.x()) {
    continue;
    } else if (state.y() + robot_params_.center_offset.y() >
            global_space_params_.max_val.y() - 0.5 * robot_box_size_.y()) {
    continue;
    } else if (state.z() + robot_params_.center_offset.z() >
            global_space_params_.max_val.z() - 0.5 * robot_box_size_.z()) {
    continue;
    }

        // Check if the voxel area is free
    if (map_manager_->getBoxStatus(Eigen::Vector3d(state[0], state[1], state[2])
       + robot_params_.center_offset, robot_box_size_, true) // True for stopping at unknown voxels
         == MapManager::VoxelStatus::kFree)  {  
      
      random_sampler_.pushSample(state, true);
      found = true;
    
    } else {
      
      random_sampler_.pushSample(state, false);
      
      stat_->num_vertices_fail++;
      
    }

  }
  return found;
}

} // prm
} // search