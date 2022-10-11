#include "prm/prm.h"

namespace search{
namespace prm{


Prm::Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){

    visualization_ = new Visualization(nh_, nh_private_);

    roadmap_graph_.reset(new GraphManager());

    map_manager_->resetMap();

    //TODO initialize odometry_ready to size of robotvector

    for (int i = 0; i < odometry_ready_.size(); i++){
        odometry_ready_[i] = false;
    }





}

Prm::GraphStatus Prm::planPath(){
  // tuning parameters
  int loop_count(0);
  int num_vertices_added(0);
  int num_edges_added(0);

  while ((loop_count++ < planning_params_.num_loops_max) && 
  (num_vertices_added < planning_num_vertices_max_) &&
  (num_edges_added < planning_num_edges_max_)) {
    StateVec new_state;
    if (!sampleVertex(new_state)) {
      continue; // skip invalid sample
    }

    ExpandGraphReport rep;
    expandGraph(roadmap_graph_, new_state, rep);
  }
}

void Prm::expandGraph(std::shared_ptr<GraphManager> graph, 
                      StateVec& new_state, ExpandGraphReport& rep){
  // Find nearest neighbour
  Vertex* nearest_vertex = NULL;
  if (!roadmap_graph_->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();
  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((direction_norm <= planning_params_.edge_length_min)) {
    // Should not add short edge.
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }
  // Recalculate the distance.
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];
  // Since we are buiding graph,
  // Consider to check the overshoot for both 2 directions except root node.
  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;

  // Check collision if lazy mode is not activated
  if (MapManager::VoxelStatus::kFree ==
      map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true) || lazy_mode_) {
    Vertex* new_vertex =
        new Vertex(roadmap_graph_->generateVertexID(), new_state);
    // Form a tree as the first step.
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    roadmap_graph_->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    roadmap_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    ++rep.num_edges_added;

    // add more edges to create graph
    std::vector<Vertex*> nearest_vertices;
    if (!roadmap_graph_->getNearestVertices(
            &new_state, planning_params_.nearest_range, &nearest_vertices)) {
      rep.status = ExpandGraphStatus::kErrorKdTree;
      return;
    }
    origin << new_vertex->state[0],new_vertex->state[1],new_vertex->state[2];
    for (int i = 0; i < nearest_vertices.size(); ++i) {
      direction << nearest_vertices[i]->state[0] - origin[0],
          nearest_vertices[i]->state[1] - origin[1],
          nearest_vertices[i]->state[2] - origin[2];
      double d_norm = direction.norm();

      if ((d_norm > planning_params_.nearest_range_min) &&
          (d_norm < planning_params_.nearest_range_max)) {
        Eigen::Vector3d p_overshoot =
            direction / d_norm * planning_params_.edge_overshoot;
        Eigen::Vector3d p_start =
            origin + robot_params_.center_offset - p_overshoot;
        Eigen::Vector3d p_end =
            origin + robot_params_.center_offset + direction;
        if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;
        // Check collision if lazy mode is not activated
        if (MapManager::VoxelStatus::kFree ==
              map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true) || lazy_mode_) {
            roadmap_graph_->addEdge(new_vertex, nearest_vertices[i], d_norm);
            ++rep.num_edges_added;
        }
      }
    }
    
  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void Prm::setState(StateVec& state, int unit_id){
  if (!odometry_ready_[unit_id]) {
    // First time receive the pose/odometry for planning purpose.
    // Reset the voxblox map
    ROS_WARN("Received the first odometry from unit %d", unit_id);
  }
  current_states_[unit_id] = state;
  odometry_ready_[unit_id] = true;
}

bool Prm::sampleVertex(StateVec& state) {
  bool found = false;
  int while_thresh = 1000; // tuning param

  while (!found && while_thresh--){
      
    random_sampler_.generate(current_vertices_[active_id_]->state, state);

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