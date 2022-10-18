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

bool Prm::loadParams(){
  std::string ns = ros::this_node::getName();

  // Load all relevant parameters
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false

  if (!free_frustum_params_.loadParams(ns + "/FreeFrustumParams")) {
    ROS_WARN("No setting for FreeFrustumParams.");
    // return false;
  }

  if (!robot_params_.loadParams(ns + "/RobotParams")) return false;

  if (!local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local")) return false;
  
  if (!planning_params_.loadParams(ns + "/PlanningParams")) return false;

  if (!random_sampler_.loadParams(ns + "/RandomSamplerParams/SamplerForSearching")) return false;

  if (!global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global")) return false;

  if (!robot_dynamics_params_.loadParams(ns + "/RobotDynamics")) return false;

  random_sampling_params_ = new RandomSamplingParams();

  planning_params_.v_max = robot_dynamics_params_.v_max;
  planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  initializeParams();
  return true;
}

void Prm::initializeParams(){
  random_sampler_.setParams(local_space_params_, global_space_params_);

  // Precompute robot box size for planning
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

  visualization_->visualizeWorkspace(current_state_, global_space_params_, local_space_params_);
}

Prm::GraphStatus Prm::planPath(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path){
  // tuning parameters
  int loop_count(0);
  int num_vertices_added(0);
  int num_edges_added(0);
  int num_target_neighbours(0);

  StateVec target_state;
  convertPoseMsgToState(target_pose, target_state);
  current_waypoints_[active_id_] = current_vertices_[active_id_];

  Eigen::Vector3d dir_vec(current_states_[active_id_][0] - target_state[0],
                          current_states_[active_id_][1] - target_state[1],
                          current_states_[active_id_][2] - target_state[2]);
  double dir_dist = dir_vec.norm();
  double best_dist = 10000000; //inf
  
  bool stop_sampling = false;

  std::vector<Vertex*> target_neighbours;

  Vertex* current_v = current_vertices_[active_id_];


  while ((!stop_sampling)&&(loop_count++ < planning_params_.num_loops_max) && 
  (num_vertices_added < planning_num_vertices_max_) &&
  (num_edges_added < planning_num_edges_max_)) {

    StateVec new_state;
    if (!sampleVertex(new_state)) {
      continue; // skip invalid sample
    }

    ExpandGraphReport rep;
    expandGraph(roadmap_graph_, new_state, rep);

    if (rep.status == ExpandGraphStatus::kSuccess) {

      num_vertices_added += rep.num_vertices_added;
      num_edges_added += rep.num_edges_added;
      // Check if state is inside target area
      Eigen::Vector3d radius_vec(new_state[0] - target_state[0],
                                 new_state[1] - target_state[1],
                                 new_state[2] - target_state[2]);
      
      // Check if sampled vertex is close enough to target area
      if (radius_vec.norm() < random_sampling_params_->reached_target_radius) {

        target_neighbours.push_back(rep.vertex_added);
        num_target_neighbours++;

        if (num_target_neighbours > random_sampling_params_->num_paths_to_target_max){
          // stop samling if we have enough sampled points in target area
          stop_sampling = true;
        }
      }

      if ((num_target_neighbours < 1) && (radius_vec.norm() < dir_dist) && (radius_vec.norm() < best_dist)) {
        // if no points in target area is sampled, we go to the point closest to the target in euclidean distance
        best_dist = radius_vec.norm();
        current_waypoints_[active_id_] = rep.vertex_added;
      }
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) && 
        (roadmap_graph_->getNumVertices() <= 1)) {
      break;
    }
  }
  ROS_INFO("Formed a graph with [%d] vertices and [%d] edges with [%d] loops, ntn[%d]",
          roadmap_graph_->getNumVertices(), roadmap_graph_->getNumEdges(), loop_count, num_target_neighbours);
  
  

  if (num_target_neighbours < 1) {
    ROS_INFO("Target not yet reached by roadmap, updated waypoint as best vertex");
  }

  std::vector<int> path_id_list;
  roadmap_graph_->findShortestPaths(current_v->id, roadmap_graph_rep_);
  // Get the shortest path to current waypoint, and collision check the path if in lazy mode
  if(lazy_mode_){

    bool collision_free_path_found = false;
    while(!collision_free_path_found){
      roadmap_graph_->getShortestPath(current_waypoints_[active_id_]->id, roadmap_graph_rep_, false, path_id_list);
      for(int i = 0; i < path_id_list.size()-1; i++){
        Vertex* v_start = roadmap_graph_->getVertex(path_id_list[i]);
        Vertex* v_end = roadmap_graph_->getVertex(path_id_list[i]);
        eigen::Vector3d p_start = 
        if (!map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)){
          // edge is not collision free
          roadmap_graph_->removeEdge(p_start, p_end);
          ROS_WARN("Collision found in path, replanning");
          roadmap_graph_->findShortestPaths(current_v, roadmap_graph_rep_);
          break;
        }
        if (i == path_id_list.size()-1) {
          // We have iterated through the whole path without having a collision
          collision_free_path_found = true;
          Prm::GraphStatus res = Prm::GraphStatus::OK;
        }
      }
    }
    if (!collision_free_path_found) {
      Prm::GraphStatus res = Prm::GraphStatus::ERR_NO_FEASIBLE_PATH;
    }
  } else {
    // Get the shortest path to current waypoint
    roadmap_graph_->getShortestPath(current_waypoints_[active_id_]->id, roadmap_graph_rep_, false, path_id_list);
  }

  // Creation of path data structure
  while (!path_id_list.empty()) {
    geometry_msgs::Pose pose;
    int id = path_id_list.back();
    path_id_list.pop_back();
    convertStateToPoseMsg(roadmap_graph_->getVertex(id)->state, pose);
    best_path.push_back(pose);
  }

  // Yaw correction
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (best_path.size() - 1); ++i) {
      Eigen::Vector3d vec(best_path[i + 1].position.x - best_path[i].position.x,
                          best_path[i + 1].position.y - best_path[i].position.y,
                          best_path[i + 1].position.z - best_path[i].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      best_path[i + 1].orientation.x = quat.x();
      best_path[i + 1].orientation.y = quat.y();
      best_path[i + 1].orientation.z = quat.z();
      best_path[i + 1].orientation.w = quat.w();
    }
  }

  visualization_->visualizeSampler(random_sampler_);
  visualization_->visualizeBestPaths(roadmap_graph_, roadmap_graph_rep_, 0, current_waypoint[active_id_]->id);

  if (roadmap_graph_->getNumVertices() > 1){
    visualization_->visualizeGraph(roadmap_graph_);
  } else {
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO("Number of failed samples: [%d] vertices and [%d] edges",
    stat_->num_vertices_fail, stat_->num_edges_fail);
    return Prm::GraphStatus res = Prm::GraphStatus::ERR_KDTREE;
  }
  return res;
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