#include "prm/prm.h"

namespace search{
namespace prm{

Prm::unit::unit(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){
 reached_final_target_ = false;
 target_status_ = Prm::StateStatus::UNINITIALIZED;
 unit_status_ = Prm::StateStatus::UNINITIALIZED;
 pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("m100/velodyne_points", 1000);
}

void Prm::unit::setOdomSubscriber(std::string odom_prefix){
  odometry_sub_ =
      nh_.subscribe("m100_" + odom_prefix + "/ground_truth/odometry_throttled", 100, &unit::odometryCallback, this);
}

void Prm::unit::setPclSubscriber(std::string pcl_prefix){
  pointcloud_sub_ = nh_.subscribe("m100_" + pcl_prefix + "/velodyne_points", 100, &unit::pclCallback, this);
}

void Prm::unit::setUnitPtr(const std::vector<Prm::unit*>& units_for_pcl){
  units_for_pcl_ = &units_for_pcl;
}

void Prm::unit::pclCallback(const sensor_msgs::PointCloud2& pcl){
  sensor_msgs::PointCloud2 pcl_filtered = pcl;

  //Iterate through pointcloud
  for (sensor_msgs::PointCloud2Iterator<float> it(pcl_filtered, "x"); it != it.end(); ++it){
    float pcl_x = it[0];
    float pcl_y = it[1];
    float pcl_z = it[2];
    
    // Iterate through units to remove points around them
    for (int i = 0; i < (*units_for_pcl_).size(); i++){
      float distance = sqrt(pow(pcl_x - (*units_for_pcl_)[i]->current_state_[0], 2) + pow(pcl_y - (*units_for_pcl_)[i]->current_state_[1], 2)
                      + pow(pcl_z - (*units_for_pcl_)[i]->current_state_[2], 2));

      if (distance < 1.5*pcl_clear_radius_){
        // Clear points around robots
        it[0] = std::numeric_limits<float>::quiet_NaN();
        it[1] = std::numeric_limits<float>::quiet_NaN();
        it[2] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  pointcloud_pub_.publish(pcl_filtered);
}

Prm::Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){


  visualization_ = new Visualization(nh_, nh_private_);
  roadmap_graph_.reset(new GraphManager());
  map_manager_ = new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
  nh_, nh_private_);
  //TODO initialize odometry_ready to size of robotvector, currently one
  odometry_ready_.push_back(false);

  active_id_ = 0;
  
  lazy_mode_ = false;

  stat_.reset(new SampleStatistic());

  if (!loadParams()) {
    ROS_ERROR("Can not load params. Shut down ROS node.");
    ros::shutdown();
  }



  for (int i = 0; i <= num_robots_; i++){
    unit* u = new unit(nh_, nh_private_);
    u->setID(i);

    std::string odom_pre = std::to_string(i);
    u->setOdomSubscriber(odom_pre);
    u->setPclSubscriber(odom_pre);
    u->setUnitPtr(units_);

    double clear_rad = robot_box_size_.norm();
    ROS_INFO("clear rad: %f", clear_rad);
    u->setClearRad(clear_rad);

    units_.push_back(u);
  }
  ROS_INFO("PRM registered %d units", units_.size());
  visualization_->visualizeWorkspace(units_[active_id_]->current_state_, global_space_params_, local_space_params_);

}

void Prm::hardReset(){
  roadmap_graph_.reset(new GraphManager());
  stat_.reset(new SampleStatistic());

}

void Prm::softReset(){
  // ROS_WARN("before statinit");
  // StateVec root_state = units_[active_id_]->current_state_;
  // ROS_WARN("middle state, x: %f, y: %f, z: %f", root_state.x(), root_state.y(), root_state.z());

  // stat_->init(root_state);
  // //--------where to put code under?
  // ROS_WARN("after statinit and before adding vertex");

  // Vertex* root_vertex = new Vertex(roadmap_graph_->generateVertexID(), root_state);
  // roadmap_graph_->addVertex(root_vertex);
  // ROS_WARN("after addvertex");

  // // First check if this position is free to go.
  // MapManager::VoxelStatus voxel_state = map_manager_->getBoxStatus(
  //   Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
  //       robot_params_.center_offset,
  //   robot_box_size_, true);
  // if (MapManager::VoxelStatus::kFree != voxel_state) {
  //   switch (voxel_state) {
  //     case MapManager::VoxelStatus::kFree:
  //       ROS_INFO("Current box is Free.");
  //       break;
  //     case MapManager::VoxelStatus::kOccupied:
  //       ROS_INFO("Current box contains Occupied voxels.");
  //       break;
  //     case MapManager::VoxelStatus::kUnknown:
  //       ROS_INFO("Current box contains Unknown voxels.");
  //       break;
  //   }
  //   // Assume that even it is not fully free, but safe to clear these voxels.
  //   ROS_WARN("Starting position is not clear--> clear space around the robot.");
  //   map_manager_->augmentFreeBox(
  //       Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
  //           robot_params_.center_offset,
  //       robot_box_size_);
  //   }

  //   visualization_->visualizeRobotState(root_vertex->state, robot_params_);
  //   visualization_->visualizeSensorFOV(root_vertex->state, sensor_params_);
  //   visualization_->visualizeWorkspace(
  //       root_vertex->state, global_space_params_, local_space_params_);
}

bool Prm::loadParams(){
  std::string ns = ros::this_node::getName();

  // Load all relevant parameters
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

  nh_.getParam(ns + "/num_robots", num_robots_);

  random_sampling_params_ = new RandomSamplingParams();

  planning_params_.v_max = robot_dynamics_params_.v_max;
  planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  initializeParams();
  return true;
}

void Prm::initializeParams(){
  random_sampler_.setParams(local_space_params_, local_space_params_);

  // Precompute robot box size for planning
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;

}

std::vector<geometry_msgs::Pose> Prm::runPlanner(geometry_msgs::Pose& target_pose){
  ros::Duration(1.0).sleep();  // sleep to unblock the thread to get update
  ros::spinOnce();
  printUnit(active_id_);
  std::vector<geometry_msgs::Pose> best_path;
  best_path.clear();
  // Find status of active unit in relation to the graph

  detectUnitStatus(active_id_);
  Prm::StateStatus unit_status = units_[active_id_]->unit_status_;

  map_manager_->augmentFreeBox(
    Eigen::Vector3d(units_[active_id_]->current_state_[0],units_[active_id_]->current_state_[1],units_[active_id_]->current_state_[2]) +
      robot_params_.center_offset,
      robot_box_size_);


  if (unit_status == Prm::StateStatus::UNINITIALIZED){
    ROS_WARN("Robot is uninitialized, some unknown error has happened");
    return best_path;
  }
  if (unit_status == Prm::StateStatus::ERROR){
    ROS_WARN("Could not add state to graph");
    return best_path;
  }
  if (unit_status == Prm::StateStatus::EMPTYGRAPH){
    ROS_INFO("Graph is empty, adding root vertex");
    addStartVertex();
  }
  if (unit_status == Prm::StateStatus::DISCONNECTED){
    ROS_INFO("Unit is disconnected from graph, adding current state as vertex for new graph segment");
    addStartVertex();
  }
  
  units_[active_id_]->reached_final_target_ = false;
  Prm::GraphStatus status = planPath(target_pose, best_path);
  if (best_path.size() == 1) {
    ROS_WARN("First x coordinateof short path: %f", best_path[0].position.x);
  }
  visualization_->visualizeRefPath(best_path);
  switch(status) {
    case Prm::GraphStatus::ERR_KDTREE:
      ROS_WARN("Error with the graph");
      break;
    case Prm::GraphStatus::ERR_NO_FEASIBLE_PATH:
      ROS_WARN("Could not find feasible path to execute");
      break;
    case Prm::GraphStatus::NOT_OK:
      ROS_WARN("Some error");
      break;
    case Prm::GraphStatus::OK:
      ROS_INFO("Found path to execute");
      break;
  }
  return best_path;
}

Prm::GraphStatus Prm::planPath(geometry_msgs::Pose& target_pose, std::vector<geometry_msgs::Pose>& best_path){
  // tuning parameters
  int loop_count(0);
  int num_vertices_added(0);
  int num_edges_added(0);
  int num_target_neighbours(0);


  StateVec target_state;
  convertPoseMsgToState(target_pose, target_state);

  units_[active_id_]->final_target_ = target_state;

  



  units_[active_id_]->current_waypoint_ = units_[active_id_]->current_vertex_;
  ROS_INFO("Current vertex: %f %f %f", units_[active_id_]->current_vertex_->state[0], units_[active_id_]->current_vertex_->state[1], units_[active_id_]->current_vertex_->state[2]);


  Eigen::Vector3d dir_vec(units_[active_id_]->current_state_[0] - target_state[0],
                          units_[active_id_]->current_state_[1] - target_state[1],
                          units_[active_id_]->current_state_[2] - target_state[2]);


  
  double dir_dist = dir_vec.norm();
  double best_dist = 10000000; //inf

  // catch if robot is already at target
  if (abs(dir_vec.norm()) < random_sampling_params_->reached_target_radius) {
    units_[active_id_]->reached_final_target_ = true;
    ROS_INFO("Unit %d already at target given", active_id_);
    return Prm::GraphStatus::OK;
  }
  
  
  bool stop_sampling = false;
  detectTargetStatus(active_id_);
  if (units_[active_id_]->target_status_ == Prm::StateStatus::CONNECTED){
    ROS_INFO("Target already connected!");
    stop_sampling = true;
    num_target_neighbours++;
  }

  std::vector<Vertex*> target_neighbours;

  stat_->init(units_[active_id_]->current_vertex_->state);

  
  ROS_INFO("Current state of unit %d: %f %f %f", active_id_, units_[active_id_]->current_state_[0], units_[active_id_]->current_state_[1], units_[active_id_]->current_state_[2]);


  while ((!stop_sampling)&&(loop_count++ < planning_params_.num_loops_max) && 
    (num_vertices_added < planning_num_vertices_max_) &&
    (num_edges_added < planning_num_edges_max_)) {
    StateVec new_state;
    if (!sampleVertex(new_state)) {
      ROS_INFO("x: %f y: %f z: :f", new_state.x(), new_state.y(), new_state.z());
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
        ROS_WARN("TARGET SAMPLED");
        target_neighbours.push_back(rep.vertex_added);
        num_target_neighbours++;
        units_[active_id_]->reached_final_target_ = true;
        //placeholder before adding pathsort
        units_[active_id_]->current_waypoint_ = target_neighbours[0];
        //
        if (num_target_neighbours > random_sampling_params_->num_paths_to_target_max){
          // stop samling if we have enough sampled points in target area
          stop_sampling = true;
        }
      }
      if ((num_target_neighbours < 1) && (radius_vec.norm() < dir_dist) && (radius_vec.norm() < best_dist)) {
        // if no points in target area is sampled, we go to the point closest to the target in euclidean distance
        best_dist = radius_vec.norm();
        units_[active_id_]->current_waypoint_ = rep.vertex_added;
      }
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) && 
        (roadmap_graph_->getNumVertices() <= 1)) {
      break;
    }
  }

  ROS_INFO("Formed a graph with [%d] vertices and [%d] edges with [%d] loops, ntn[%d]",
          roadmap_graph_->getNumVertices(), roadmap_graph_->getNumEdges(), loop_count, num_target_neighbours);
  
  Prm::GraphStatus res = Prm::GraphStatus::NOT_OK;

  if(roadmap_graph_->getNumVertices() < 2){
    ROS_WARN("Sampler failed, try again");
    return res;
  }

  if (num_target_neighbours < 1) {
    ROS_INFO("Target not yet reached by roadmap, updated waypoint as best vertex");
  }
  ROS_INFO("Currentv: %d, x: %f", units_[active_id_]->current_vertex_->id, units_[active_id_]->current_vertex_->state.x());
  std::vector<int> path_id_list;
  roadmap_graph_rep_.reset();
  roadmap_graph_->findShortestPaths(units_[active_id_]->current_vertex_->id, roadmap_graph_rep_);
  // Get the shortest path to current waypoint, and collision check the path if in lazy mode
 
  if(lazy_mode_){

    bool collision_free_path_found = false;
    while(!collision_free_path_found){
      roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, false, path_id_list);
      for(int i = 0; i < path_id_list.size()-1; i++){
        Vertex* v_start = roadmap_graph_->getVertex(path_id_list[i]);
        Vertex* v_end = roadmap_graph_->getVertex(path_id_list[i+1]);
        
        if (!checkCollisionBetweenVertices(v_start, v_end)){
          // edge is not collision free
          roadmap_graph_->removeEdge(v_start, v_end);
          ROS_WARN("Collision found in path, replanning");
          roadmap_graph_->findShortestPaths(units_[active_id_]->current_vertex_->id, roadmap_graph_rep_);
          break;
        }
        if (i == path_id_list.size()-1) {
          // We have iterated through the whole path without having a collision
          collision_free_path_found = true;
          res = Prm::GraphStatus::OK;
        }
      }
    }
    if (!collision_free_path_found) {
      res = Prm::GraphStatus::ERR_NO_FEASIBLE_PATH;
    }
  } else {
    // Get the shortest path to current waypoint
    roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, false, path_id_list);
    res = Prm::GraphStatus::OK;
 
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
  //visualization_->visualizeBestPaths(roadmap_graph_, roadmap_graph_rep_, 0, units_[active_id_]->current_waypoint_->id);
  if (roadmap_graph_->getNumVertices() > 1){
    visualization_->visualizeGraph(roadmap_graph_);
    ROS_WARN("viz graph");
  } else {
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO("Number of failed samples: [%d] vertices and [%d] edges",
    stat_->num_vertices_fail, stat_->num_edges_fail);
    res = Prm::GraphStatus::ERR_KDTREE;
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
  
  
  if ( MapManager::VoxelStatus::kFree ==
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



void Prm::addStartVertex(){


  Vertex* root_vertex = new Vertex(roadmap_graph_->generateVertexID(), units_[active_id_]->current_state_);
  
  map_manager_->augmentFreeBox(
    Eigen::Vector3d(root_vertex->state[0], root_vertex->state[1], root_vertex->state[2]) +
        robot_params_.center_offset,
    robot_box_size_);

  roadmap_graph_->addVertex(root_vertex);

  units_[active_id_]->current_vertex_ = root_vertex;

}

void Prm::detectUnitStatus(int unit_id){
  if (roadmap_graph_->getNumVertices() < 1) {
    ROS_INFO("Graph is empty");
    units_[unit_id]->unit_status_ = Prm::StateStatus::EMPTYGRAPH;
    return;
  }
  StateVec cur_state;
  cur_state << units_[unit_id]->current_state_.x(), units_[unit_id]->current_state_.y(),
                        units_[unit_id]->current_state_.z(), units_[unit_id]->current_state_.w();
  Vertex* nearest;
  if (roadmap_graph_->getNearestVertexInRange(&cur_state, planning_params_.edge_length_min, &nearest)){
    ROS_INFO("Unit %d is on a vertex", unit_id);
    units_[unit_id]->current_vertex_ = nearest;
    units_[unit_id]->unit_status_ = Prm::StateStatus::CONNECTED;
    return;
  }
  if (roadmap_graph_->getNearestVertexInRange(&cur_state, planning_params_.nearest_range_max, &nearest)){
    ROS_INFO("Unit %d is near a vertex", unit_id);
    Vertex* link_vertex = NULL;
    bool connected_to_graph = connectStateToGraph(roadmap_graph_, units_[unit_id]->current_state_, link_vertex, 0.5);
    if (connected_to_graph){
      units_[unit_id]->current_vertex_ = link_vertex;
      units_[unit_id]->unit_status_ = Prm::StateStatus::CONNECTED;
      ROS_INFO("Successfully added current state of unit %d to graph", unit_id);
    } else {
      units_[unit_id]->unit_status_ = Prm::StateStatus::ERROR;
      ROS_WARN("Could not successfully add current state of %d to graph", unit_id);
    }
    return;
  } else {
    units_[unit_id]->unit_status_ = Prm::StateStatus::DISCONNECTED;
    ROS_WARN("Unit %d is to far from the established graph, start a new graph segment", unit_id);
  }
  
}

void Prm::detectTargetStatus(int unit_id){
  if (roadmap_graph_->getNumVertices() < 1) {
    ROS_INFO("Graph is empty");
    units_[unit_id]->target_status_ = Prm::StateStatus::EMPTYGRAPH;
    return;
  }
  StateVec tar_state;
  tar_state << units_[unit_id]->final_target_.x(), units_[unit_id]->final_target_.y(),
                        units_[unit_id]->final_target_.z(), units_[unit_id]->final_target_.w();
  Vertex* nearest;
  if (roadmap_graph_->getNearestVertexInRange(&tar_state, random_sampling_params_->reached_target_radius, &nearest)){
    ROS_INFO("Vertex exist in target %d area", unit_id);
    units_[unit_id]->current_waypoint_ = nearest;
    units_[unit_id]->target_status_ = Prm::StateStatus::CONNECTED;
    return;
  }
  if (roadmap_graph_->getNearestVertexInRange(&tar_state, planning_params_.nearest_range_max, &nearest)){
    ROS_INFO("Target %d is near a vertex", unit_id);
    Vertex* link_vertex = NULL;
    bool connected_to_graph = connectStateToGraph(roadmap_graph_, units_[unit_id]->final_target_, link_vertex, 0.5);
    if (connected_to_graph){
      units_[unit_id]->current_waypoint_ = link_vertex;
      units_[unit_id]->target_status_ = Prm::StateStatus::CONNECTED;
      ROS_INFO("Successfully added current target of unit %d to graph", unit_id);
    } else {
      units_[unit_id]->target_status_ = Prm::StateStatus::ERROR;
      ROS_WARN("Could not successfully add current target of %d to graph", unit_id);
    }
    return;
  } else {
    units_[unit_id]->target_status_ = Prm::StateStatus::DISCONNECTED;
    ROS_WARN("Target %d is too far from the established graph, start sampler to expand", unit_id);
    return;
  }
  
}




void Prm::setActiveUnit(int unit_id){
  active_id_ = unit_id;
}



bool Prm::sampleVertex(StateVec& state) {
  bool found = false;
  int while_thresh = 1000; // tuning param

  while (!found && while_thresh--){
      
    random_sampler_.generate(units_[active_id_]->current_vertex_->state, state);

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
    //ROS_INFO("x: %f, y: %f, z: %f", state.x(), state.y(), state.z());

        // Check if the voxel area is free
    if (map_manager_->getBoxStatus(Eigen::Vector3d(state[0], state[1], state[2])
       + robot_params_.center_offset, robot_box_size_, true) // True for stopping at unknown voxels
         == MapManager::VoxelStatus::kFree)  {  
      found = true;
      random_sampler_.pushSample(state, found);
      
      
    } else {
      found = false;
      random_sampler_.pushSample(state, found);
    }

  }
  
  return found;
}

bool Prm::checkCollisionBetweenVertices(Vertex* v_start, Vertex* v_end){
  // Get start position
  Eigen::Vector3d origin(v_start->state[0], v_start->state[1],
                         v_start->state[2]);
  // Get edge direction
  Eigen::Vector3d direction(v_end->state[0] - origin[0], v_end->state[1] - origin[1],
                            v_end->state[2] - origin[2]);
  
  // Add robot overshoot
  Eigen::Vector3d overshoot_vec = planning_params_.edge_overshoot * direction.normalized();
  
  // Add overshoot in start position except root
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (v_start->id != 0) start_pos = start_pos - overshoot_vec;
  // Add overshoot in end position
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;
  
  // Check collision along edge
  if(MapManager::VoxelStatus::kFree == map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true)){
    return true;
  }

  return false;

}

void Prm::expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                        Vertex* new_vertex, ExpandGraphReport& rep){

  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state),
                                         planning_params_.nearest_range,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1],
        nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    if ((d_norm > planning_params_.edge_length_min) &&
        (d_norm < planning_params_.edge_length_max)) {
      Eigen::Vector3d p_overshoot =
          direction / d_norm * planning_params_.edge_overshoot;
      Eigen::Vector3d p_start =
          origin + robot_params_.center_offset - p_overshoot;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
      if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;
      if (MapManager::VoxelStatus::kFree ==
          map_manager_->getPathStatus(p_start, p_end, robot_box_size_, true)) {
        graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
        ++rep.num_edges_added;
      }
    }
  }
  rep.status = ExpandGraphStatus::kSuccess;

}

bool Prm::connectStateToGraph(std::shared_ptr<GraphManager> graph,
                              StateVec& cur_state, Vertex*& v_added,
                              double dist_ignore_collision_check) {
  Vertex* nearest_vertex = NULL;
  if (!graph->getNearestVertex(&cur_state, &nearest_vertex)) return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();
  bool connect_state_to_graph = true;
  const double kDelta = 0.05;
  if (direction_norm <= kDelta) {
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(graph, nearest_vertex, rep);
    v_added = nearest_vertex;
  } else if (direction_norm <= std::max(dist_ignore_collision_check,
                                        planning_params_.edge_length_min)) {
    // Blindly add a link/vertex to the graph if small radius.
    Vertex* new_vertex = new Vertex(graph->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    graph->addVertex(new_vertex);
    graph->addEdge(new_vertex, nearest_vertex, direction_norm);
    // Add edges only from this vertex.
    ExpandGraphReport rep;
    expandGraphEdges(graph, new_vertex, rep);
    v_added = new_vertex;
  } else {
    ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
    ExpandGraphReport rep;
    expandGraph(graph, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN("[GlobalGraph] Added successfully.");
      v_added = rep.vertex_added;
    } else {
      // Not implemented solution for this case yet.
      connect_state_to_graph = false;
      ROS_WARN("[GlobalGraph] Can not add current state to graph since: ");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN("kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN("kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN("kErrorShortEdge.");
          break;
      }
    }
  }
  return connect_state_to_graph;
}

bool Prm::getTargetReachedSingle(int unit_id) {
  return units_[unit_id]->reached_final_target_;
}

int Prm::getActiveUnit(){
  return active_id_;
}

void Prm::printUnit(int unit_id){
  ROS_INFO("Unit id: %d", units_[unit_id]->id_);
  ROS_INFO("Active id: %d", active_id_);
  ROS_INFO("Current state is x: %f, y: %f, z: %f", units_[unit_id]->current_state_[0], units_[unit_id]->current_state_[1], units_[unit_id]->current_state_[2]);
}

} // prm
} // search