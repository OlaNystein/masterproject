#include "rrtstar/rrtstar.h"

namespace search {
namespace rrtstar{


RobotStateHistory::RobotStateHistory() {
  kd_tree_ = NULL;
  reset();
}

void RobotStateHistory::reset() {
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
}

void RobotStateHistory::addState(StateVec* s){
  kd_insert3(kd_tree_, s->x(), s->y(), s->z(), s);
  state_hist_.push_back(s);
}

bool RobotStateHistory::getNearestState(const StateVec* state, StateVec** s_res) {
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
      kd_res_free(nearest);
      return false;  
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool RobotStateHistory::getNearestStates(const StateVec* state, double range,
                                         std::vector<StateVec*>* s_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  if (state_hist_.size() == 0) return false;
  kdres* neighbors = kd_nearest_range3(kd_tree_, 
      state->x(), state->y(), state->z(), range); // Visualize volumetric gain.
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  s_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    StateVec* new_neighbor = (StateVec*)kd_res_item_data(neighbors);
    s_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool RobotStateHistory::getNearestStateInRange(const StateVec* state,
                                               double range, StateVec** s_res) {
  if (state_hist_.size() == 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = (StateVec*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*s_res)->x(), state->y() - (*s_res)->y(),
      state->z() - (*s_res)->z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

Rrtstar::Rrtstar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){
  map_manager_ = new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
    nh_, nh_private_);
  
  visualization_ = new Visualization(nh_, nh_private_);

  tree_.reset(new GraphManager());

  odometry_ready = false;

}

void Rrtstar::reset(){
  // Reset the graphs
  tree_.reset(new GraphManager());
  tree_rep_.reset();

  // Re-initialize data structs
  stat_.reset(new SampleStatistic());

  // Set state for root/source vertex.
  StateVec root_state;
  root_state = current_state_;
  stat_->init(root_state);

  // Create a root vertex and add to the graph.
  // Root vertex should be assigned id 0.
  root_vertex_ = new Vertex(tree_->generateVertexID(), root_state);
  tree_->addVertex(root_vertex_);

  // Check if position is in free space
  MapManager::VoxelStatus voxel_state = map_manager_->getBoxStatus(
      Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) + 
          robot_params_.center_offset, robot_box_size_, true);
  if (MapManager::VoxelStatus::kFree != voxel_state) {
    switch (voxel_state) {
      case MapManager::VoxelStatus::kOccupied:
        ROS_INFO("Current box contains occupied voxels");
        break;
      case MapManager::VoxelStatus::kUnknown:
        ROS_INFO("Current box contains unknown voxels");
        break;
    }
    ROS_WARN("Starting position is not clear--> clear space around the robot");
    map_manager_->augmentFreeBox(
      Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
        robot_params_.center_offset, robot_box_size_);
  } else {
    ROS_INFO("Current box is free");
  }

  // Clear free space before planning
  if (planning_params_.free_frustum_before_planning) {
    map_manager_->augmentFreeFrustum();
  }
  visualization_->visualizeRobotState(root_vertex_->state, robot_params_);
  visualization_->visualizeSensorFOV(root_vertex_->state, sensor_params_);

  visualization_->visualizeWorkspace(root_vertex_->state, global_space_params_, local_space_params_);
}

bool Rrtstar::loadParams(){
  std::string ns = ros::this_node::getName();

  //Load all relevant parameters
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false;

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

void Rrtstar::initializeParams(){
  // Placeholder because function takes to inputs. TODO: make one-input version
  random_sampler_.setParams(local_space_params_, local_space_params_);

  // Precompute the robot box for planning.
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

  visualization_->visualizeWorkspace(current_state_, global_space_params_, local_space_params_);
}

void Rrtstar::setState(StateVec& state){
  if (!odometry_ready) {
    // First time receive the pose/odometry for planning purpose.
    // Reset the octomap
    ROS_WARN("Received the first odometry, reset the map");
    map_manager_->resetMap();
  }
  current_state_ = state;
  odometry_ready = true;
}

Rrtstar::TreeStatus Rrtstar::buildTree(){
  int loop_count(0);
  int num_vertices(1);
  int num_edges(0);

  while ((loop_count++ < planning_params_.num_loops_max) &&
  (num_vertices < planning_num_vertices_max_) &&
  (num_edges < planning_num_edges_max_)) {

    StateVec new_state;
    if (!sampleVertex(new_state)) {
      continue; // skip invalid sample state
    }

    ExpandGraphReport rep;
    expandTree(tree_, new_state, rep);

    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added; // Always one in tree
      num_edges += rep.num_edges_added; // Always one in tree
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) &&
        (tree_->getNumVertices() <= 1)) {
      break; // Early cutoff if no vertices are added (something very wrong)
    }
  }
  ROS_INFO("Formed a graph with [%d] vertices and [%d] edges with [%d] loops",
           num_vertices, num_edges, loop_count);

  visualization_->visualizeSampler(random_sampler_);
  if (tree_->getNumVertices() > 1) {
    visualization_->visualizeGraph(tree_);
    return Rrtstar::TreeStatus::OK;
  } else {
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO("Number of failed samples: [%d] vertices and [%d] edges",
            stat_->num_vertices_fail, stat_->num_edges_fail);
    return Rrtstar::TreeStatus::ERR_NO_FEASIBLE_PATH;
  }
}


bool Rrtstar::sampleVertex(StateVec& state){
  bool found = false;
  int while_thres = 1000; // magic number
  while (!found && while_thres--) {
    random_sampler_.generate(root_vertex_->state, state);
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

void Rrtstar::expandTree(std::shared_ptr<GraphManager> tree, StateVec& new_state, ExpandGraphReport& rep){
  //------new_state is the random sampled state----
  //------find the nearest vertex -----------------
  //--------------NEAREST-----------------------
  Vertex* nearest_vertex = NULL;
  if (!tree->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  //-----------NEAREST-------------------------

  //--------STEER------------------------------------
  // Find the edge length and checking its between max and min (tuning parameters)
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                          nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0]-origin[0], new_state[1]-origin[1],
                            new_state[2]-origin[2]);
  // Find edge length
  double dir_norm = direction.norm();
  // If its over max edge length, shorten it to maximum
  if (dir_norm > planning_params_.edge_length_max){
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if (dir_norm <= planning_params_.edge_length_min) { 
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }

  // Calculate new edge length
  dir_norm = direction.norm();
  // Modify the new state to fit inside edge boundary
  new_state[0] = origin[0] + direction[0]; //sampled = nearestV + correct dir
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];
//----------/STEER---------------------------------

  // Finding the robot overshoot
  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;

  // Expanding the actual tree
  // ------------ if OBSTACLEFREE
  if (MapManager::VoxelStatus::kFree == 
      map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true)) {
    
    //-----NEAR----------
    std::vector<Vertex*> nearest_vertices;
    if (!tree_->getNearestVertices(&new_state, planning_params_.nearest_range, &nearest_vertices)) {
      rep.status = ExpandGraphStatus::kErrorKdTree;
      return;
    }

    std::vector<Vertex*> feasible_nearest_vertices;

    Vertex* v_min = nearest_vertex;
    double c_min = v_min->distance + dir_norm;
    origin[0] = new_state[0];
    origin[1] = new_state[1];
    origin[2] = new_state[2]; //Change origin to new state to find min-cost-path

    // Get the least cost feasible neighbour
    for (int i = 0; i < nearest_vertices.size(); i++){
      direction[0] = nearest_vertices[i]->state[0] - new_state[0];
      direction[1] = nearest_vertices[i]->state[1] - new_state[1];
      direction[2] = nearest_vertices[i]->state[2] - new_state[2];
      dir_norm = direction.norm();
      if (dir_norm == 0.0){
        continue;
      }

      Eigen::Vector3d path_start = origin + robot_params_.center_offset;
      Eigen::Vector3d path_end = origin + robot_params_.center_offset + direction;

      if (map_manager_->getPathStatus(path_start, path_end, robot_box_size_, true) == 
          MapManager::VoxelStatus::kFree){
        feasible_nearest_vertices.push_back(nearest_vertices[i]);
        double cost_tmp = dir_norm + nearest_vertices[i]->distance;
        if (cost_tmp < c_min) {
          v_min = nearest_vertices[i];
          c_min = cost_tmp;
        }
      }
    }

    // Add the new vertex to the tree

    Vertex* new_vertex = new Vertex(tree_->generateVertexID(), new_state);
    new_vertex->parent = v_min;
    new_vertex->distance = c_min;
    v_min->children.push_back(new_vertex);
    tree_->addVertex(new_vertex);
    rep.num_vertices_added++;
    tree_->addEdge(new_vertex, v_min, c_min - v_min->distance);
    rep.num_edges_added++;


    // Rewire the tree if the new found vertex yields shorter path to near vertex
    for (auto& near_vertex : feasible_nearest_vertices) {
      direction[0] = near_vertex->state[0] - new_state[0];
      direction[1] = near_vertex->state[1] - new_state[1];
      direction[2] = near_vertex->state[2] - new_state[2];
      dir_norm = direction.norm();
      if (dir_norm == 0.0){
        continue;
      }

      double cost_tmp = dir_norm + new_vertex->distance;
      if (cost_tmp < near_vertex->distance) { //check if path through new vertex is better
        tree_->removeEdge(near_vertex, near_vertex->parent);
        near_vertex->distance = cost_tmp;
        near_vertex->parent = new_vertex;
        tree_->addEdge(near_vertex, near_vertex->parent, dir_norm);
      }
    }
  } else { // edge path in collision area
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2], end_pos[0],
                                    end_pos[1], end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

ConnectStatus Rrtstar::search(geometry_msgs::Pose source_pose,
                      geometry_msgs::Pose target_pose, 
                      std::vector<geometry_msgs::Pose>& best_path){
  // Converting poses
  StateVec source;
  convertPoseMsgToState(source_pose, source_state);
  StateVec target;
  convertPoseMsgToState(target_pose, target_state);

  // Some initialization
  ConnectStatus search_result;
  best_path.clear();
  tree_->reset();

  ROS_INFO("Starting to search for a path from src [%f,%f,%f] to tgt [%f,%f,%f]", source[0],
           source[1], source[2], target[0], target[1], target[2]);

  // Verify source is collision free
  MapManager::VoxelStatus voxel_state;
  voxel_state = map_manager_->getBoxStatus(Eigen::Vector3d(
    source[0], source[1], source[2]) + robot_params_.center_offset,
    robot_box_size_, true);

  if (voxel_state != MapManager::VoxelStatus::kFree) {
    switch (voxel_state) {
      case MapManager::VoxelStatus::kOccupied:
        ROS_WARN("Source position contains occupied voxels --> abort search");
        break;
      case MapManager::VoxelStatus::kUnknown:
        ROS_WARN("Source position contains unknown voxels --> abort search");
        break;
    }
    search_result = ConnectStatus::kErrorCollisionAtSource;
    return search_result;
  }

  // Initialize the tree with our unoccupied source
  Vertex* source_vertex = new Vertex(tree_->generateVertexID(), source);
  tree_->addVertex(source);
  root_vertex_ = source_vertex;
    
  // Sample points and add to tree
  bool target_reached = false;
  std::vector<Vertex*> target_neighbours;
  int loop count = 0;
  int num_vertices = 0;
  int num_edges = 0;
  random_sampler_.reset();
  bool stop_sampling = false;

  while (!stop_sampling) {
    StateVec new_state;
    if(!sampleVertex(new_state)) continue;
    ExpandGraphReport rep;
    expandGraph(tree_, new_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      // Check if state is neighbour of target
      Eigen::Vector3d radius_vec(new_state[0] - target[0],
                                 new_state[1] - target[1]
                                 new_state[2] - target[2]);
      if (radius_vec.norm() < random_sampling_params_->reached_target_radius) {
        target_neighbours.push_back(rep.vertex_added);
        target_reached = true;
        // We have reached the target without collision

      }
    }
  }
  

}

}
}