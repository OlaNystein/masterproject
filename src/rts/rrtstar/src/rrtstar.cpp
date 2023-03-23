#include "rrtstar/rrtstar.h"

namespace search {
namespace rrtstar{


Rrtstar::Rrtstar(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){
  map_manager_ = new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
    nh_, nh_private_);
  
  visualization_ = new Visualization(nh_, nh_private_);

  tree_.reset(new GraphManager());

  tree_old_.reset(new GraphManager());

  odometry_ready = false;
  stuck_ = false;

  current_search_iteration_ = 0;

  total_distance_ = 0;
  avg_graph_build_time_ = 0;
  avg_waypoint_finder_time_ = 0;

  rostime_start_ = ros::Time::now();

  StateVec root_state;
  root_state = current_state_;
  root_vertex_ = new Vertex(tree_->generateVertexID(), root_state);

  free_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud", 10);

  free_pointcloud_update_timer_ =
      nh_.createTimer(ros::Duration(kFreePointCloudUpdatePeriod),
                      &Rrtstar::freePointCloudtimerCallback, this);
}

void Rrtstar::reset(){
  // Reset the graphs
  tree_.reset(new GraphManager());
  tree_old_.reset(new GraphManager());
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
  // if (MapManager::VoxelStatus::kFree != voxel_state) {
  //   switch (voxel_state) {
  //     case MapManager::VoxelStatus::kOccupied:
  //       ROS_INFO("Current box contains occupied voxels");
  //       break;
  //     case MapManager::VoxelStatus::kUnknown:
  //       ROS_INFO("Current box contains unknown voxels");
  //       break;
  //   }
  //   ROS_WARN("Starting position is not clear--> clear space around the robot");
  //   map_manager_->augmentFreeBox(
  //     Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
  //       robot_params_.center_offset, robot_box_size_);
  // } else {
  //   ROS_INFO("Current box is free");
  // }

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
      continue; // skip invalid sampled state
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
           tree_->getNumVertices(), num_edges, loop_count);
  
  tree_->findShortestPaths(tree_rep_);
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

Rrtstar::TreeStatus Rrtstar::buildTree(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path){
  
  int loop_count(0);
  int num_vertices(1);
  int num_edges(0);
  
  current_search_iteration_++;
  if(!(current_search_iteration_ == 1)) {
    this->reset(); //reset tree if planner is in iteration mode
  } 

  bool target_reached = false;
  std::vector<Vertex*> target_neighbours;
  int num_paths_to_target = 0;
  bool stop_sampling = false;
  StateVec target_state;
  convertPoseMsgToState(target_pose, target_state);
  current_waypoint_ = root_vertex_;

  
  Eigen::Vector3d opt_vec(current_state_[0] - target_state[0],
                          current_state_[1] - target_state[1],
                          current_state_[2] - target_state[2]);
  double opt_dist = opt_vec.norm();
  ROS_INFO("Opt distance: %f", opt_dist);
  
  MapManager::VoxelStatus targetStatus = map_manager_->getBoxStatus(Eigen::Vector3d(target_state[0], target_state[1], target_state[2])
       + robot_params_.center_offset, robot_box_size_, true); // True for stopping at unknown voxels


  START_TIMER(btime);

  while (((loop_count++ < planning_params_.num_loops_max) &&
  (num_vertices < planning_num_vertices_max_) && (!stop_sampling))||
  ((targetStatus == MapManager::VoxelStatus::kFree)&&(!target_reached)&&loop_count<planning_params_.num_loops_cutoff)) {
 
    StateVec new_state;
    if (!sampleVertex(new_state)) {
      //ROS_WARN("Skipping invalid sample");
      continue; // skip invalid sample state
    }

    ExpandGraphReport rep;
    expandTree(tree_, new_state, rep);
    
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added; // 
      num_edges += rep.num_edges_added; // 
      // Check if state is inside target area
      Eigen::Vector3d radius_vec(new_state[0] - target_state[0],
                                 new_state[1] - target_state[1],
                                 new_state[2] - target_state[2]);
      if (radius_vec.norm() < random_sampling_params_->reached_target_radius) {
        target_neighbours.push_back(rep.vertex_added);
        target_reached = true;
        final_target_reached_ = true;
        ROS_INFO("Final dist to target: %f", radius_vec.norm());
        // We have reached the target without collision
        num_paths_to_target++;
        current_waypoint_ = target_neighbours[0];
        if (num_paths_to_target > random_sampling_params_->num_paths_to_target_max) {
          stop_sampling = true;
        }
      }
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) &&
        (tree_->getNumVertices() <= 1)) {
      break; // Early cutoff if no vertices are added (something very wrong)
    }
  }
  
  avg_graph_build_time_ += GET_ELAPSED_TIME(btime);
 
  ROS_INFO("Formed a graph with [%d] vertices and [%d] edges with [%d] loops, nptt[%d]",
           tree_->getNumVertices(), tree_->getNumEdges(), loop_count, num_paths_to_target);
 
  tree_->findShortestPaths(tree_rep_);
  tree_->findLeafVertices(tree_rep_);
  
  
  if (!target_reached) {
    ROS_INFO("Finding best commited path to next waypoint");
    double best_dist = 10000000; //infinity

    // get all vertices of current tree (rewired)
  
    Vertex* leaf_vertice = nullptr; 
    
    // find the leaf node with the lowest heuristic value
    int num_v = tree_->getNumVertices();

    for (int i = 1; i < num_v; i++) { //iterate through all vertices to find best candidate
      leaf_vertice = tree_->getVertex(i);
      Eigen::Vector3d vec_to_target(target_state[0] - leaf_vertice->state[0],
                            target_state[1]- leaf_vertice->state[1],
                            target_state[2] - leaf_vertice->state[2]);

      double norm_to_target = vec_to_target.norm();
      double dist = norm_to_target;
      Eigen::Vector3d vertex_pos(leaf_vertice->state[0],leaf_vertice->state[1],leaf_vertice->state[2]);
      
      if ((dist < best_dist)&&(dist<opt_dist)&&(map_manager_->getBoxStatus(vertex_pos, robot_params_.safety_extension,true) == MapManager::VoxelStatus::kFree)){
        current_waypoint_ = leaf_vertice;
        best_dist = dist;
        // update next waypoint with the best vertex
      }
    }
   } else {
     current_waypoint_ = target_neighbours[0];
   }
  if ((current_waypoint_ == root_vertex_)&&(!target_reached)){
    ROS_INFO("I'm stuck");
    return Rrtstar::TreeStatus::STUCK;
  }

  // Get the shortest path to current waypoint
  std::vector<int> path_id_list;
  tree_->getShortestPath(current_waypoint_->id, tree_rep_, false, path_id_list);
  total_distance_ = total_distance_ + current_waypoint_->distance;
  ROS_INFO("Printing current waypoint: x: %f, y: %f z: %f", current_waypoint_->state.x(), current_waypoint_->state.y(), current_waypoint_->state.z());

  // Creation of the path data structure
  while (!path_id_list.empty()) {
    geometry_msgs::Pose pose;
    int id = path_id_list.back();
    path_id_list.pop_back();
    convertStateToPoseMsg(tree_->getVertex(id)->state, pose);
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

  //Visualization
  visualization_->visualizeSampler(random_sampler_);
  visualization_->visualizeBestPaths(tree_, tree_rep_, 0, current_waypoint_->id);
  
  if (final_target_reached_) { //Timer results
    avg_graph_build_time_ = avg_graph_build_time_/current_search_iteration_;
    avg_waypoint_finder_time_ = avg_waypoint_finder_time_/current_search_iteration_;
    ROS_INFO("Iterations: %d", current_search_iteration_);
    ROS_INFO("TotalDistance: %f, AvgBuild: %f", total_distance_, avg_graph_build_time_);
    current_search_iteration_ = 0;
  }
  
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
    //ROS_INFO("State: x: %f, y: %f, z: %f", state[0], state[1], state[2]);
    
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
  //-----------/NEAREST-------------------------
  
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
      //ROS_WARN("Error with kdtree in expansion. ");
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
    rep.vertex_added = new_vertex;


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

bool Rrtstar::buildTreeFromNewRoot(Vertex* root){
  tree_old_ = tree_;
  tree_.reset(new GraphManager());
  //tree_ = std::make_shared<GraphManager>(*tree_old_);
  Vertex* new_root = new Vertex(tree_->generateVertexID(), root->state);
  stat_.reset(new SampleStatistic());
  stat_->init(new_root->state);
  tree_->addVertex(new_root);

  root_vertex_ = new_root;
  ROS_INFO("New source vertex id: %d, v: %d", root_vertex_->id, tree_->getNumVertices());

  // for (auto &v: tree_->vertices_map_){
  //   // Reset values
  //   v.second->distance = 0;
  //   v.second->is_leaf_vertex = false;
  //   // Remove children pointers
  //   for (auto &u: v.second->children) {
  //     tree_->removeEdge(v.second, u);
  //   }
  //   // Remove parent from all other than root
  //   if (v.second->id != 0){
  //     tree_->removeEdge(v.second->parent, v.second);
  //   }
    
  //   // Change ID of previous root
  //   if (v.second->id == 0){
  //     v.second->id = root->id;
  //   }
  // }
  int oldsuccesses = 0;
  int oldfailures = 0;
  for (auto &v : tree_old_->vertices_map_){
    if(!(v.second->id == root->id)){
      ExpandGraphReport rep;
      expandTree(tree_, v.second->state, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        oldsuccesses++;
      } else {
        oldfailures++;
      }
    }
  }

  ROS_INFO("Rerooting resulted in %d successes and %d failures", oldsuccesses, oldfailures);

  if (oldsuccesses < 2){
    return false;
  }
  // Set new root ID to zero
  // root->id = 0;
  // tree_rep_.reset();
  // tree_->findShortestPaths(tree_rep_);

  return true;
}

double Rrtstar::getEuclidean(Vertex* v, Vertex* u){
  Eigen::Vector3d origin = Eigen::Vector3d(v->state[0], v->state[1], v->state[2]);
  Eigen::Vector3d target = Eigen::Vector3d(u->state[0], u->state[1], u->state[2]);
  double origin_norm = origin.norm();
  double target_norm = target.norm();
  return abs(origin_norm - target_norm);
}

bool Rrtstar::search(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path, std::shared_ptr<GraphManager> treet, ShortestPathsReport& tree_rep){
  // Converting poses
  StateVec source_state = root_vertex_->state;

  StateVec target_state;
  convertPoseMsgToState(target_pose, target_state);
  
  // Some initialization
  ConnectStatus search_result;
  best_path.clear();
  //tree_->reset();

  ROS_INFO("Starting to search for a path from src [%f,%f,%f] to tgt [%f,%f,%f]", source_state[0],
           source_state[1], source_state[2], target_state[0], target_state[1], target_state[2]);

  int num_vertices = treet->getNumVertices();
  
  
  // Verify source is collision free
  MapManager::VoxelStatus voxel_state;
  voxel_state = map_manager_->getBoxStatus(Eigen::Vector3d(
    source_state[0], source_state[1], source_state[2]) + robot_params_.center_offset,
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
    //return search_result;
    return false;
  }
  
  // Initialize the tree with our unoccupied source
  Vertex* source_vertex = new Vertex(treet->generateVertexID(), source_state);
  treet->addVertex(source_vertex);
  root_vertex_ = source_vertex;
  
  // Sample points and add to tree
  bool target_reached = false;
  std::vector<Vertex*> target_neighbours;
  int loop_count = 0;

  int num_edges = 0;
  random_sampler_.reset();
  bool stop_sampling = false;
  int num_paths_to_target = 0;
  ROS_INFO("1.1");
  while (!stop_sampling) {
    StateVec new_state;
    
    if(!sampleVertex(new_state)) continue;
    ExpandGraphReport rep;
    
    expandTree(treet, new_state, rep);
    
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      // Check if state is neighbour of target
      Eigen::Vector3d radius_vec(new_state[0] - target_state[0],
                                 new_state[1] - target_state[1],
                                 new_state[2] - target_state[2]);
      if (radius_vec.norm() < random_sampling_params_->reached_target_radius) {
        target_neighbours.push_back(rep.vertex_added);
        target_reached = true;
        // We have reached the target without collision
        num_paths_to_target++;
        if (num_paths_to_target > random_sampling_params_->num_paths_to_target_max) {
          stop_sampling = true;
        }
      }
    }
   
    if ((loop_count >= random_sampling_params_->num_loops_cutoff) && (treet->getNumVertices() <= 1)) {
      stop_sampling = true;
    }
  
    if ((loop_count++ > random_sampling_params_->num_loops_max) ||
          (num_vertices > random_sampling_params_->num_vertices_max) ||
          (num_edges > random_sampling_params_->num_edges_max)){
      stop_sampling = true;
    }
  }
  ROS_INFO("Expanded a tree with currently %d vertices and %d edges + nptt: %d", num_vertices, num_edges, num_paths_to_target);
  visualization_->visualizeGraph(treet);
  visualization_->visualizeSampler(random_sampler_);
  ROS_INFO("v: %d", treet->getNumVertices());

  // bool added_target = false;
  // Vertex* target_vertex = NULL;
  // if (target_reached) {
  //   final_target_reached_ = true;
  //   ROS_WARN("Reached target.");
  //   voxel_state = map_manager_->getBoxStatus(Eigen::Vector3d(target_state[0], target_state[1], target_state[2]) + 
  //                 robot_params_.center_offset,
  //                 robot_box_size_, true);
  //   if (voxel_state == MapManager::VoxelStatus::kFree) {
  //     ExpandGraphReport rep;
  //     expandTree(tree, target_state, rep);
     
  //     if (rep.status == ExpandGraphStatus::kSuccess) {
  //       ROS_INFO("Added target to the graph successfully.");
  //       num_vertices += rep.num_vertices_added;
  //       num_edges += rep.num_edges_added;
  //       added_target = true;
  //       target_vertex = rep.vertex_added;
  //     } else {
  //       ROS_INFO("Cannot expand the graph to connect to the target.");
  //     }
  //   } else {
  //     ROS_INFO("Target is not free, failed to add to graph. Only target neighbour is reachable");
  //   }
  // } else {
  //   ROS_INFO ("Target not reached yet");
  // }
  ROS_INFO("h1");
  tree_rep.reset();
  ROS_INFO("h2");
  treet->findShortestPaths(tree_rep);
  
  ROS_INFO("h3");
  target_reached = true;
  if (!target_reached) {
    ROS_INFO("Finding best commited path to next waypoint");
    double best_dist = 10000000;
    // get leaf vertices of current tree (rewired)
    treet->getLeafVertices(leaf_vertices_);
    // find the leaf node with the lowest heuristic value
    for (auto& leaf_vertice : leaf_vertices_) {
      Eigen::Vector3d vec_to_target(target_state[0] - leaf_vertice->state[0],
                            target_state[1]- leaf_vertice->state[1],
                            target_state[2] - leaf_vertice->state[2]);
      double norm_to_target = vec_to_target.norm();
      double dist = leaf_vertice->distance + norm_to_target;
      if (dist < best_dist){
        current_waypoint_ = leaf_vertice;
        // update next waypoint with the best vertex
      }
    }
   } //else if (!added_target && target_reached){

  //   ROS_INFO("Sorting best path to target neighbour. ");
  //   std::sort(target_neighbours.begin(), target_neighbours.end(),
  //             [&tree, &tree_rep](const Vertex* a, const Vertex* b) {
  //               return tree->getShortestDistance(a->id, tree_rep) <
  //                      tree->getShortestDistance(b->id, tree_rep);
  //             });
  //   ROS_INFO("h4");
  //   current_waypoint_ = target_neighbours[0];
  // }
  ROS_INFO("h5");
  std::vector<int> path_id_list;
  treet->getShortestPath(current_waypoint_->id, tree_rep, false, path_id_list);
  
  current_waypoint_id_ = current_waypoint_->id;

  // convert each state in path to pose messages
  while (!path_id_list.empty()) {
    geometry_msgs::Pose pose;
    int id = path_id_list.back();
    path_id_list.pop_back();
    convertStateToPoseMsg(treet->getVertex(id)->state, pose);
    best_path.push_back(pose);
  }

  // Set the heading angle tangent with the moving direction,
  // from the second waypoint; the first waypoint keeps the same direction.
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
  

  ROS_WARN("Finish searching. ");
  search_result = ConnectStatus::kSuccess;
  visualization_->visualizeBestPaths(treet, tree_rep, 0, current_waypoint_id_);

  return true;//search_result;
}

std::vector<geometry_msgs::Pose> Rrtstar::runSearch(geometry_msgs::Pose& target_pose){
  ros::Duration(1.0).sleep();  // sleep to unblock the thread to get update
  ros::spinOnce();

  std::vector<geometry_msgs::Pose> best_path;
  
  final_target_reached_ = false;
  if (current_search_iteration_ == 0){
    tree_->reset();
    tree_old_->reset();
    tree_rep_.reset();
    stuck_ = false;
    stat_.reset(new SampleStatistic());
    StateVec root_state;
    root_state = current_state_;
    root_vertex_ = new Vertex(tree_->generateVertexID(), root_state);
    stat_->init(root_state);
    tree_->addVertex(root_vertex_);
    ROS_INFO("Source vertex id: %d, v: %d", root_vertex_->id, tree_->getNumVertices());
    ROS_INFO("Current state: %f %f %f", current_state_[0], current_state_[1], current_state_[2]);
  }
  TreeStatus stat = buildTree(target_pose, best_path);
  if (stat == TreeStatus::NOT_OK){ //(stat == ConnectStatus::kErrorCollisionAtSource) {
    ROS_WARN("Something wrong with search");
    return best_path;
  } else if (stat == TreeStatus::ERR_NO_FEASIBLE_PATH) {//ConnectStatus::kErrorNoFeasiblePath) {
    ROS_WARN("Found no feasible path to execute");
    return best_path;
  } else if (stat == TreeStatus::STUCK) {
    ROS_WARN("The robot is stuck, aborting planner");
    stuck_ = true;
    return best_path;
  }
  ROS_INFO("Search successful, found next segment. ");
  stuck_ = false;
  return best_path;
}

bool Rrtstar::getTargetStatus() {
  return final_target_reached_;
}

bool Rrtstar::getStuckStatus() {
  return stuck_;
}

void Rrtstar::freePointCloudtimerCallback(const ros::TimerEvent& event) {
  
  if (!planning_params_.freespace_cloud_enable) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_body(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<Eigen::Vector3d> multiray_endpoints_body;
  for (auto sensor_name : free_frustum_params_.sensor_list) {
    StateVec state;
    state[0] = current_state_[0];
    state[1] = current_state_[1];
    state[2] = current_state_[2];
    state[3] = current_state_[3];
    // get frustum endpoints (They are in world frame)
    free_frustum_params_.sensor[sensor_name].getFrustumEndpoints(
        state, multiray_endpoints_body);
    std::vector<Eigen::Vector3d> multiray_endpoints;
    // Check it the full ray till max range is free(for voxblox only, for
    // octomap just convert to world frame)
    map_manager_->getFreeSpacePointCloud(multiray_endpoints_body, state,
                                         free_cloud_body);
    // convert the endpoint to sensor frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    free_frustum_params_.sensor[sensor_name].convertBodyToSensor(
        free_cloud_body, free_cloud);

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*free_cloud.get(), out_cloud);
    out_cloud.header.frame_id =
        free_frustum_params_.sensor[sensor_name].frame_id;
    out_cloud.header.stamp = ros::Time::now();
    free_cloud_pub_.publish(out_cloud);
  }
}

void Rrtstar::rayCast(StateVec& state) {

  // Get global bounds
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;

  if(global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] = global_space_params_.min_val[i] +
                     global_space_params_.min_extension[i];
      bound_max[i] = global_space_params_.max_val[i] +
                     global_space_params_.max_extension[i];
    }
  } else {
    ROS_ERROR("Global space is sphere or undefined. ");
    return;
  }

  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;
  // refine bound inside sensor range
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ind++){
    std::string sensor_name = planning_params_.exp_sensor_list[ind];

    for (int i = 0; i < 3; i++) {
      bound_min[i] = std::max(bound_min[i], 
                              state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] = std::min(bound_max[i], 
                              state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;

    map_manager_->getScanStatus(origin, multiray_endpoints,
                                voxel_log_tmp);
    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      MapManager::VoxelStatus vs = vl.second;
      if (vs == MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      int j = 0;
      for (j = 0; j < 3; j++) {
        if ((voxel[j] < bound_min[j]) || (voxel[j] > bound_max[j])) break;
      }
      if (j == 3) {
        // valid voxel.
        if (vs == MapManager::VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == MapManager::VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == MapManager::VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR("Unsupported voxel type.");
        }
        voxel_log.push_back(std::make_pair(voxel, vs));
      }
    }
    visualization_->visualizeRays(state, multiray_endpoints);
    visualization_->visualizeMap(bound_min, bound_max, voxel_log,
                                            map_manager_->getResolution());
  }

}

}
}