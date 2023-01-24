#include "prm/prm.h"

namespace search{
namespace prm{

Prm::unit::unit(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private), tf_listener_(nh){
 reached_final_target_ = false;
 target_status_ = Prm::StateStatus::UNINITIALIZED;
 unit_status_ = Prm::StateStatus::UNINITIALIZED;
 publish_throttle_it_ = 10;
 pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("m100/velodyne_points", 10);
}

void Prm::unit::setOdomSubscriber(std::string odom_prefix){
  odometry_sub_ =
      nh_.subscribe("m100_" + odom_prefix + "/ground_truth/odometry_throttled", 100, &unit::odometryCallback, this);
}

void Prm::unit::setPclSubscriber(std::string pcl_prefix){
  pointcloud_sub_ = nh_.subscribe("m100_" + pcl_prefix + "/velodyne_points", 10, &unit::pclCallback, this);
}

void Prm::unit::setUnitPtr(const std::vector<Prm::unit*>& units_for_pcl){
  units_for_pcl_ = &units_for_pcl;
}

void Prm::unit::pclCallback(const sensor_msgs::PointCloud2& pcl){
  publish_throttle_it_++;
  if(publish_throttle_it_ > 9){
    publish_throttle_it_ = 0;

    sensor_msgs::PointCloud2 pcl_filtered = pcl;

    tf::StampedTransform sensor_to_world_transform;
    tf_listener_.lookupTransform("world", pcl_filtered.header.frame_id, pcl_filtered.header.stamp, sensor_to_world_transform);

    //Iterate through pointcloud
    for (sensor_msgs::PointCloud2Iterator<float> it(pcl_filtered, "x"); it != it.end(); ++it){


      // float pcl_x = it[0];
      // float pcl_y = it[1];
      // float pcl_z = it[2];
      tf::Vector3 point(it[0], it[1], it[2]);
      point = sensor_to_world_transform(point);
      float pcl_x = point.x();
      float pcl_y = point.y();
      float pcl_z = point.z();
      
      // Iterate through units to remove points around them
      for (int i = 0; i < (*units_for_pcl_).size(); i++){
        float distance = sqrt(pow(pcl_x - (*units_for_pcl_)[i]->current_state_[0], 2) + pow(pcl_y - (*units_for_pcl_)[i]->current_state_[1], 2)
                        + pow(pcl_z - (*units_for_pcl_)[i]->current_state_[2], 2));

        if (distance < 1*pcl_clear_radius_){
          // Clear points around robots
          // it[0] = std::numeric_limits<float>::quiet_NaN();
          // it[1] = std::numeric_limits<float>::quiet_NaN();
          // it[2] = std::numeric_limits<float>::quiet_NaN();
          it[0] = 0;
          it[1] = 0;
          it[2] = 0;
        }
      }
    }

    pointcloud_pub_.publish(pcl_filtered);
  }
}

Prm::Prm(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){


  roadmap_graph_.reset(new GraphManager());
  map_manager_ = new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
  nh_, nh_private_);
  //TODO initialize odometry_ready to size of robotvector, currently one
  odometry_ready_.push_back(false);

  minimap_ = new Minimap(nh, nh_private);

  active_id_ = 0;
  
  lazy_mode_ = false;

  stat_.reset(new SampleStatistic());

  if (!loadParams()) {
    ROS_ERROR("Can not load params. Shut down ROS node.");
    ros::shutdown();
  }



  for (int i = 0; i < num_robots_; i++){
    unit* u = new unit(nh_, nh_private_);
    u->setID(i);

    std::string odom_pre = std::to_string(i);
    u->setOdomSubscriber(odom_pre);
    u->setPclSubscriber(odom_pre);
    u->setUnitPtr(units_);

    double clear_rad = robot_box_size_.norm();
    u->setClearRad(clear_rad);

    units_.push_back(u);
    std::pair<int, StateVec*> p = std::make_pair(u->id_, &(u->current_state_));
    cur_states_.push_back(p);

    Visualization* v = new Visualization(nh_, nh_private_);
    visualization_.push_back(v);
    v->setId(u->id_);
  }
  minimap_->setStatePtr(cur_states_);
  visualization_[active_id_]->visualizeWorkspace(units_[active_id_]->current_state_, global_space_params_, local_space_params_);

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

  //   visualization_[active_id_]->visualizeRobotState(root_vertex->state, robot_params_);
  //   visualization_[active_id_]->visualizeSensorFOV(root_vertex->state, sensor_params_);
  //   visualization_[active_id_]->visualizeWorkspace(
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

  if (!robot_dynamics_params_.loadParams("pci_mav_ros_node/RobotDynamics")) return false;

  nh_.getParam(ns + "/num_robots", num_robots_);

  random_sampling_params_ = new RandomSamplingParams();

  planning_params_.v_max = robot_dynamics_params_.v_max;
  planning_params_.v_homing_max = robot_dynamics_params_.v_homing_max;

  initializeParams();
  return true;
}

void Prm::initializeParams(){
  random_sampler_.setParams(global_space_params_, local_space_params_);

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
  }
  visualization_[active_id_]->visualizeRefPath(best_path);
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

  std::vector<geometry_msgs::Pose> best_path_temp;

  StateVec target_state;
  convertPoseMsgToState(target_pose, target_state);

  units_[active_id_]->final_target_ = target_state;

  



  units_[active_id_]->current_waypoint_ = units_[active_id_]->current_vertex_;
  //ROS_INFO("Current vertex: %f %f %f", units_[active_id_]->current_vertex_->state[0], units_[active_id_]->current_vertex_->state[1], units_[active_id_]->current_vertex_->state[2]);


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
    units_[active_id_]->reached_final_target_ = true;
  }

  std::vector<Vertex*> target_neighbours;

  stat_->init(units_[active_id_]->current_vertex_->state);

  
  //ROS_INFO("Current state of unit %d: %f %f %f", active_id_, units_[active_id_]->current_state_[0], units_[active_id_]->current_state_[1], units_[active_id_]->current_state_[2]);
  // Eigen
  // random_sampler_.setBound()

  while ((!stop_sampling)&&(loop_count++ < planning_params_.num_loops_max) && 
    (num_vertices_added < planning_num_vertices_max_) &&
    (num_edges_added < planning_num_edges_max_)) {
    StateVec new_state;
    if (!sampleVertex(new_state)) {
      //ROS_INFO("x: %f y: %f z: :f", new_state.x(), new_state.y(), new_state.z());
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
        //ROS_WARN("TARGET SAMPLED");
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
    Vertex* waypoint;
    roadmap_graph_->getNearestVertex(&target_state, &waypoint);
    units_[active_id_]->current_waypoint_ = waypoint;
    ROS_INFO("Target not yet reached by roadmap, updated waypoint as best vertex");
  }
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
    double traverse_length = 0;
    double traverse_time = 0;
    std::vector<StateVec> best_path_states;
    roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, true, best_path_states);
    Eigen::Vector3d p0(best_path_states[0][0], best_path_states[0][1], best_path_states[0][2]);
    std::vector<Vertex*> best_path_vertices;
    roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, true,
                                  best_path_vertices);

    const double kLenMin = 1.0;
    std::vector<Eigen::Vector3d> path_vec;
    roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, true,
                                  path_vec);
    double total_len = Trajectory::getPathLength(path_vec);
    if (total_len <= kLenMin) {
      ROS_WARN("Best path is too short.");
      return Prm::GraphStatus::ERR_NO_FEASIBLE_PATH;
    }

    for (int i = 0; i < best_path_states.size(); ++i) {
      Eigen::Vector3d p1(best_path_states[i][0], best_path_states[i][1], best_path_states[i][2]);
      Eigen::Vector3d dir_vec = p1 - p0;
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, best_path_states[i][3]);
      tf::Vector3 origin(best_path_states[i][0], best_path_states[i][1], best_path_states[i][2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      best_path_temp.push_back(pose);
      double seg_length = (p1 - p0).norm();
      traverse_length += seg_length;
      if ((traverse_length > planning_params_.traverse_length_max)) {
        break;
      }
      p0 = p1;
    }

    roadmap_graph_->getShortestPath(units_[active_id_]->current_waypoint_->id, roadmap_graph_rep_, true, path_id_list);
    
    res = Prm::GraphStatus::OK;
  }


  // collision checking
  /*ROS_INFO("Path checking of unit %d, is %d long", active_id_, path_id_list.size());
  for (auto& u: units_){
    if ((u->id_ != active_id_)){
      //ROS_INFO("helo1. %d", u->current_path_id_list_.size());
      
      double active_time = ros::Time::now().toSec();
      double active_time_segment = 0;
      // Iterate through segments in the current units path
      for(int v_active_id = 0; v_active_id < path_id_list.size() - 1; v_active_id++){
        // get first vertex in segment
        Vertex* v0 = roadmap_graph_->getVertex(path_id_list[v_active_id]);
        // get second vertex in segment
        Vertex* v1 = roadmap_graph_->getVertex(path_id_list[v_active_id+1]);
        // convert to vector
        Eigen::Vector3d p0(v0->state.x(), v0->state.y(), v0->state.z());
        // convert to vector
        Eigen::Vector3d p1(v1->state.x(), v1->state.y(), v1->state.z());
        // calculate segment vector
        Eigen::Vector3d active_segment = p1 - p0;
        active_time_segment = active_segment.norm()/planning_params_.v_max;
        //ROS_INFO("ats: %f", active_segment.norm());
        // timestamp start of segment
        double t0 = active_time;

        active_time += active_time_segment;
        // timestamp end of segment
        double t1 = active_time;
        //ROS_INFO("t0: %f, t1: %f", t0, t1);
        //---------------------Make segment-cuboid for collisionchecking----------------
        Eigen::Vector3d active_center = (p1+p0)/2;
        //ROS_INFO("acenter: x: %f, y: %f, z: %f", active_center[0], active_center[1], active_center[2]);
        Eigen::Vector3d active_half_dim = robot_box_size_/2;
        Eigen::Vector3d active_min_point = active_center - active_half_dim;
        Eigen::Vector3d active_max_point = active_center + active_half_dim;
        Eigen::AlignedBox3d active_cuboid(active_min_point, active_max_point);
        

          
        if((u->currently_moving_)&& (!u->current_path_id_list_.empty())){
        // Variable to keep tracked of checked time
        double checked_time = u->moving_time_start_.toSec();
        double checked_time_segment = 0;
        // iterate through segments in all other unit paths
        for(int v_check_id = 0; v_check_id < u->current_path_id_list_.size() - 1; v_check_id++){
          //ROS_INFO("helo2");
          // get first vertex in segment
          Vertex* vc0 = roadmap_graph_->getVertex(u->current_path_id_list_[v_check_id]);
          // get second vertex in segment
          Vertex* vc1 = roadmap_graph_->getVertex(u->current_path_id_list_[v_check_id+1]);
          // convert to vector
          Eigen::Vector3d pc0(vc0->state.x(), vc0->state.y(), vc0->state.z());
          // convert to vector
          Eigen::Vector3d pc1(vc1->state.x(), vc1->state.y(), vc1->state.z());
          // calculate segment vector
          Eigen::Vector3d check_segment = pc1 - pc0;
          // get the duration the other unit has been moving
          //double time_diff = active_time - u_start_time;
          // predict where the unit would be if it only moves along this segment
          //Eigen::Vector3d predicted_segment = pc0 + (time_diff-checked_time)*planning_params_.v_max*check_segment.normalized();
          // Add the time the robot has taken to complete the segment
          //ROS_INFO("cts: %f", check_segment.norm());
          checked_time_segment = check_segment.norm()/planning_params_.v_max;
          // timestamp start of segment
          double tc0 = checked_time;
          checked_time += checked_time_segment;
          // timestamp end of segment
          double tc1 = checked_time;
          //ROS_INFO("tc0: %f, tc1: %f", tc0, tc1);


          //---------------------Make segment-cuboid for collisionchecking----------------
          Eigen::Vector3d check_center = (pc1+pc0)/2;
          //ROS_INFO("ccenter: x: %f, y: %f, z: %f", check_center[0], check_center[1], check_center[2]);
          Eigen::Vector3d check_half_dim = robot_box_size_/2;
          Eigen::Vector3d check_min_point = check_center - check_half_dim;
          Eigen::Vector3d check_max_point = check_center + check_half_dim;
          Eigen::AlignedBox3d check_cuboid(check_min_point, check_max_point);

          if(!doCuboidsIntersect(active_cuboid, check_cuboid)){
            //The segments with robot-size does not intersect, hence collision checking not required
            //ROS_INFO("cuboidpass");
            continue;
          }
          // If we go here, it means that a segment in the best path found by the planner potentially
          // crosses with a path that is currently being executed by another unit
          // and that the unit we may collide with has not executed the scary part yet
          // We finally checks if the timing seems scary to see if the active robot should wait or not
          if (!((t1 <= tc0)||(t0 >= tc1))){
          //if(((abs(t1-tc1 < 1)) || abs(t0-tc0) < 1)){
            // The timing ranges are scary
            ROS_WARN("Planner detected possible collision, requeue at later time");
            return Prm::GraphStatus::ERR_NO_FEASIBLE_PATH;
          }
          //ROS_INFO("timepass");
          
        }
      } else if (!(u->currently_moving_)){
      //check if path passes through idle robot
      //---------------------Make segment-cuboid for collisionchecking----------------
          Eigen::Vector3d check_center(u->current_state_.x(), u->current_state_.y(), u->current_state_.z());
          
          //ROS_INFO("ccenter: x: %f, y: %f, z: %f", check_center[0], check_center[1], check_center[2]);
          Eigen::Vector3d check_half_dim = robot_box_size_/2;
          Eigen::Vector3d check_min_point = check_center - check_half_dim;
          //ROS_INFO("cdim: x: %f, y: %f, z: %f", check_half_dim[0], check_half_dim[1], check_half_dim[2]);
          Eigen::Vector3d check_max_point = check_center + check_half_dim;
          Eigen::AlignedBox3d check_cuboid(check_min_point, check_max_point);

        

          if(doCuboidsIntersect(active_cuboid, check_cuboid)){
            //The path passes through an idle robot
            ROS_INFO("idle collision");
            return Prm::GraphStatus::ERR_NO_FEASIBLE_PATH;
          }
          //ROS_INFO("idle pass");

        }
      }
    } 
  }
  */


  units_[active_id_]->current_path_id_list_ = path_id_list;
  if (!(best_path_temp.empty())){
    double yawhalf = units_[active_id_]->current_state_[3] * 0.5;
    best_path_temp[0].orientation.x = 0.0;
    best_path_temp[0].orientation.y = 0.0;
    best_path_temp[0].orientation.z = sin(yawhalf);
    best_path_temp[0].orientation.w = cos(yawhalf);
  }
  for (int i = 0; i < (best_path_temp.size() - 1); ++i) {
      Eigen::Vector3d vec(best_path_temp[i + 1].position.x - best_path_temp[i].position.x,
                          best_path_temp[i + 1].position.y - best_path_temp[i].position.y,
                          best_path_temp[i + 1].position.z - best_path_temp[i].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      best_path_temp[i + 1].orientation.x = quat.x();
      best_path_temp[i + 1].orientation.y = quat.y();
      best_path_temp[i + 1].orientation.z = quat.z();
      best_path_temp[i + 1].orientation.w = quat.w();
  }

  best_path = best_path_temp;





  //visualization_[active_id_]->visualizeSampler(random_sampler_);
  visualization_[active_id_]->visualizeBestPaths(roadmap_graph_, roadmap_graph_rep_, 0, units_[active_id_]->current_waypoint_->id);
  if (roadmap_graph_->getNumVertices() > 1){
    visualization_[active_id_]->visualizeGraph(roadmap_graph_);
  } else {
    visualization_[active_id_]->visualizeFailedEdges(stat_);
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
        map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, false) || lazy_mode_) {
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
    bool connected_to_graph = connectStateToGraph(roadmap_graph_, units_[unit_id]->current_state_, link_vertex, 0.7);
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
    bool connected_to_graph = connectStateToGraph(roadmap_graph_, units_[unit_id]->final_target_, link_vertex, 2*random_sampling_params_->reached_target_radius);
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

bool Prm::doCuboidsIntersect(const Eigen::AlignedBox3d &cuboid1, const Eigen::AlignedBox3d &cuboid2)
{
    return (cuboid1.intersects(cuboid2) || cuboid2.intersects(cuboid1));
}


// Path improvers from gbplanner

// bool Prm::addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
//                             const std::vector<geometry_msgs::Pose>& path) {
//   if (path.size() <= 0) return false;

//   // The whole path is collision free already and start from root vertex.
//   // We only need to link the first vertex to the existing graph.
//   // Then add the whole path to the graph.
//   // Finally, add more edges along the path to the graph.

//   StateVec first_state;
//   first_state << path[0].position.x, path[0].position.y, path[0].position.z,
//       0.0;
//   Vertex* nearest_vertex = NULL;
//   if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
//     return false;
//   if (nearest_vertex == NULL) return false;
//   Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
//                          nearest_vertex->state[2]);
//   Eigen::Vector3d direction(first_state[0] - origin[0],
//                             first_state[1] - origin[1],
//                             first_state[2] - origin[2]);
//   double direction_norm = direction.norm();
//   Vertex* parent_vertex = NULL;
//   const double kDeltaLimit = 0.1;
//   const double kRadiusLimit = 0.5;

//   // Add root vertex first.
//   if (direction_norm <= kDeltaLimit) {
//     parent_vertex = nearest_vertex;
//   } else if (direction_norm <=
//              std::max(kRadiusLimit, planning_params_.edge_length_min)) {
//     // @TODO: find better way to do this.
//     // Blindly add a link/vertex to the graph.
//     Vertex* new_vertex =
//         new Vertex(graph_manager->generateVertexID(), first_state);
//     new_vertex->parent = nearest_vertex;
//     new_vertex->distance = nearest_vertex->distance + direction_norm;
//     nearest_vertex->children.push_back(new_vertex);
//     graph_manager->addVertex(new_vertex);
//     graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
//     parent_vertex = new_vertex;
//   } else {
//     ROS_WARN("[GlobalGraph] Try to add current state to the graph.");
//     ExpandGraphReport rep;
//     expandGraph(graph_manager, first_state, rep);
//     if (rep.status == ExpandGraphStatus::kSuccess) {
//       ROS_WARN("[GlobalGraph] Added successfully.");
//       parent_vertex = rep.vertex_added;
//     } else {
//       ROS_WARN("[GlobalGraph] Can not add current state to the global graph.");
//       return false;
//     }
//   }

//   // Add all remaining vertices of the path.
//   std::vector<Vertex*> vertex_list;
//   vertex_list.push_back(parent_vertex);
//   for (int i = 1; i < path.size(); ++i) {
//     StateVec new_state;
//     new_state << path[i].position.x, path[i].position.y, path[i].position.z,
//         0.0;
//     Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
//                            parent_vertex->state[2]);
//     Eigen::Vector3d direction(new_state[0] - origin[0],
//                               new_state[1] - origin[1],
//                               new_state[2] - origin[2]);
//     double direction_norm = direction.norm();

//     Vertex* new_vertex =
//         new Vertex(graph_manager->generateVertexID(), new_state);
//     // new_vertex->type = vertices[i]->type;
//     new_vertex->parent = parent_vertex;
//     new_vertex->distance = parent_vertex->distance + direction_norm;
//     parent_vertex->children.push_back(new_vertex);
//     graph_manager->addVertex(new_vertex);
//     graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
//     vertex_list.push_back(new_vertex);
//     parent_vertex = new_vertex;
//   }

//   // Build edges around vertices if possible to get better path.
//   int n_vertices = 0;
//   int n_edges = 0;
//   // Assume the path is verified collision free.
//   for (int i = 0; i < vertex_list.size(); ++i) {
//     int num_vertices_added = 0;
//     int num_edges_added = 0;
//     ExpandGraphReport rep;
//     expandGraphEdges(graph_manager, vertex_list[i], rep);
//     if (rep.status == ExpandGraphStatus::kSuccess) {
//       n_vertices += num_vertices_added;
//       n_edges += num_edges_added;
//     } else {
//       switch (rep.status) {
//         case ExpandGraphStatus::kErrorKdTree:
//           ROS_WARN("Can not add this vertex: kErrorKdTree.");
//           break;
//         case ExpandGraphStatus::kErrorCollisionEdge:
//           ROS_WARN("Can not add this vertex: kErrorCollisionEdge.");
//           break;
//         case ExpandGraphStatus::kErrorShortEdge:
//           ROS_WARN("Can not add this vertex: kErrorShortEdge.");
//           break;
//       }
//     }
//   }

//   const bool path_intp_add = true;
//   const double intp_len = 1.0;  // m
//   if (path_intp_add) {
//     // Add some intermidiate vertices along the path to densify the global
//     // graph.
//     for (int i = 0; i < (vertex_list.size() - 1); ++i) {
//       Eigen::Vector3d start_vertex(vertex_list[i]->state.x(),
//                                    vertex_list[i]->state.y(),
//                                    vertex_list[i]->state.z());
//       Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(),
//                                  vertex_list[i + 1]->state.y(),
//                                  vertex_list[i + 1]->state.z());
//       Eigen::Vector3d edge_vec = end_vertex - start_vertex;
//       double edge_length = edge_vec.norm();
//       if (edge_length <= intp_len) continue;
//       edge_vec.normalize();
//       int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
//       Vertex* prev_vertex = vertex_list[i];
//       double acc_len = 0;
//       for (int j = 1; j < n_intp; ++j) {
//         Eigen::Vector3d new_v;
//         new_v = start_vertex + j * intp_len * edge_vec;
//         StateVec new_state;
//         new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3];
//         Vertex* new_vertex =
//             new Vertex(graph_manager->generateVertexID(), new_state);
//         graph_manager->addVertex(new_vertex);
//         graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
//         prev_vertex = new_vertex;
//         acc_len += intp_len;
//       }
//       // Link the last connection
//       double last_edge_len = edge_length - acc_len;
//       graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
//     }
//   }
//   return true;
// }

// bool Prm::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
//                           std::vector<geometry_msgs::Pose>& path_mod) {
//   // Few heuristics to improve the path.
//   // a) Shorten path by reducing intermidiate nodes. (be careful with turning
//   // cases) Shorten/reduce some very short paths to prevent small motion and
//   // sudden change in angles.
//   // b) Adjust nodes to its neighbors to improve safety
//   // c) Ignore leaf node of the path to prevent the robot to come too close the
//   // obstacle

//   if (path_orig.empty()) return false;

//   // Feature a) Remove short intermidiate vertices.
//   std::vector<geometry_msgs::Pose> path_mod1 = path_orig;

//   const double kSegmentLenMin = 0.5;
//   bool cont_refine = true;
//   while (cont_refine) {
//     cont_refine = false;
//     for (int i = 0; i < (path_mod1.size() - 2); ++i) {
//       Eigen::Vector3d p_start(path_mod1[i].position.x, path_mod1[i].position.y,
//                               path_mod1[i].position.z);
//       Eigen::Vector3d p_int(path_mod1[i + 1].position.x,
//                             path_mod1[i + 1].position.y,
//                             path_mod1[i + 1].position.z);
//       Eigen::Vector3d p_end(path_mod1[i + 2].position.x,
//                             path_mod1[i + 2].position.y,
//                             path_mod1[i + 2].position.z);
//       Eigen::Vector3d segment = p_int - p_start;
//       double segment_len = segment.norm();
//       // ROS_WARN("Segment length %f.", segment_len);
//       if (segment_len < kSegmentLenMin) {
//         if ((MapManager::VoxelStatus::kFree ==
//              map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
//                                          true))) {
//           // ignore the intermidiate nore, combine the first to the last node.
//           ROS_WARN("Combine nodes to remove short segments.");
//           path_mod1.erase(path_mod1.begin() + i + 1);
//           cont_refine = true;
//           break;
//         }
//       }
//     }
//   }

//   // Implement (b) first: form a safe corridor along each path from cutting
//   // hyperplanes.
//   geometry_msgs::Pose pose0;
//   pose0.position.x = path_mod1[0].position.x;
//   pose0.position.y = path_mod1[0].position.y;
//   pose0.position.z = path_mod1[0].position.z;
//   pose0.orientation.x = path_mod1[0].orientation.x;
//   pose0.orientation.y = path_mod1[0].orientation.y;
//   pose0.orientation.z = path_mod1[0].orientation.z;
//   pose0.orientation.w = path_mod1[0].orientation.w;
//   path_mod.push_back(pose0);
//   bool mod_success = true;
//   for (int i = 1; i < path_mod1.size(); ++i) {
//     Eigen::Vector3d p0(path_mod1[i - 1].position.x, path_mod1[i - 1].position.y,
//                        path_mod1[i - 1].position.z);
//     Eigen::Vector3d p0_mod(path_mod[i - 1].position.x,
//                            path_mod[i - 1].position.y,
//                            path_mod[i - 1].position.z);
//     Eigen::Vector3d p1(path_mod1[i].position.x, path_mod1[i].position.y,
//                        path_mod1[i].position.z);
//     Eigen::Vector3d p1_parallel = p0_mod + p1 - p0;

//     Eigen::Vector3d p2;
//     bool do_check_p2 = false;
//     if (i < path_mod1.size() - 1) {
//       do_check_p2 = true;
//       p2 = Eigen::Vector3d(path_mod1[i + 1].position.x,
//                            path_mod1[i + 1].position.y,
//                            path_mod1[i + 1].position.z);
//     }

//     Eigen::Vector3d p1_target = p1;
//     bool seg_free = true;
//     // if (MapManager::VoxelStatus::kFree ==
//     //     map_manager_->getPathStatus(p0_mod, p1_parallel, robot_box_size_,
//     //                                 true)) {
//     //   p1_target = p1_parallel;
//     // } else if (MapManager::VoxelStatus::kFree ==
//     //            map_manager_->getPathStatus(p0_mod, p1, robot_box_size_, true)) {
//     //   p1_target = p1;
//     // } else {
//     //   seg_free = false;
//     // }

//     Eigen::Vector3d p1_mod;
//     geometry_msgs::Pose pose;

//     Eigen::Vector3d p_center;
//     p_center = (p0_mod + p1_target) / 2.0;
//     Eigen::Vector3d p_dir;
//     p_dir = (p1 - p0);
//     double radius = p_dir.norm() / 2.0;
//     // add a local bounding box
//     Eigen::Vector3d local_bbx(2 * (radius + robot_params_.safety_extension[0]),
//                               2 * robot_params_.safety_extension[1],
//                               2 * robot_params_.safety_extension[2]);
//     std::vector<Eigen::Vector3d> occupied_voxels;
//     std::vector<Eigen::Vector3d> free_voxels;
//     map_manager_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx,
//                                            occupied_voxels, free_voxels);

//     pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(
//         new pcl::PointCloud<pcl::PointXYZ>());
//     for (auto& v : occupied_voxels) {
//       obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
//     }

//     if (seg_free && (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod))) {
//       // Verify collision-free again with the map 
//       ROS_INFO("yes");
//       if ((MapManager::VoxelStatus::kFree !=
//            map_manager_->getPathStatus(p0_mod, p1_mod, robot_box_size_,
//                                        true)) ||
//           (do_check_p2 &&
//            ((MapManager::VoxelStatus::kFree !=
//              map_manager_->getPathStatus(p1_mod, p2, robot_box_size_, true))))) {
//         //p1_mod = p1;
//         //mod_success = false;
//         ROS_WARN("Newly modified path is not collision-free.");
//         //break;  // break to save time @recheck
//       }
//     } else {
//       ROS_INFO("no");
//       //p1_mod = p1;
//       // mod_success = false;
//       // break; // break to save time @recheck
//     }
//     pose.position.x = p1_mod.x();
//     pose.position.y = p1_mod.y();
//     pose.position.z = p1_mod.z();
//     path_mod.push_back(pose);
//   }

//   // Correct the heading angle tangent with the moving direction again.
//   // Re-Assign the first heading
//   path_mod[0].orientation.x = path_orig[0].orientation.x;
//   path_mod[0].orientation.y = path_orig[0].orientation.y;
//   path_mod[0].orientation.z = path_orig[0].orientation.z;
//   path_mod[0].orientation.w = path_orig[0].orientation.w;
//   if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
//     for (int i = 1; i < path_mod.size(); ++i) {
//       Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
//                           path_mod[i].position.y - path_mod[i - 1].position.y,
//                           path_mod[i].position.z - path_mod[i - 1].position.z);
//       double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
//       path_mod[i].orientation.x = 0.0;
//       path_mod[i].orientation.y = 0.0;
//       path_mod[i].orientation.z = sin(yawhalf);
//       path_mod[i].orientation.w = cos(yawhalf);
//     }
//   }

//   return mod_success;
// }

// bool Prm::modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
//                      Eigen::Vector3d& p0, Eigen::Vector3d& p1,
//                      Eigen::Vector3d& p1_mod) {
//   p1_mod = p1;
//   Eigen::Vector3d p_center;
//   p_center = (p0 + p1) / 2.0;
//   Eigen::Vector3d p_dir;
//   p_dir = (p1 - p0);
//   double radius = p_dir.norm() / 2.0;
//   Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
//   Eigen::Quaternion<double> quat_W2S;
//   Eigen::Vector3d p_dir_norm = p_dir.normalized();
//   double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
//   double pitch_angle = -std::atan2(p_dir_norm.z(),
//                         std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
//                         p_dir_norm.y() * p_dir_norm.y()));
//   quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
//              Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

//   pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
//   Eigen::Translation<double, 3> trans_W2S(p_center);
//   Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
//   pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());

//   // Add a local bounding box
//   double kDx = robot_params_.safety_extension[0];
//   double kDy = robot_params_.safety_extension[1];
//   double kDz = robot_params_.safety_extension[2];

//   // 6 rectanges in form:  ax+by+cz = 1
//   std::vector<Eigen::Vector3d> u_l;
//   std::vector<Eigen::Vector3d> p_l;
//   u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
//   u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
//   u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
//   u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
//   u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
//   u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
//   p_l.push_back(Eigen::Vector3d(-radius - kDx, 0.0, 0.0));
//   p_l.push_back(Eigen::Vector3d(radius + kDx, 0.0, 0.0));
//   p_l.push_back(Eigen::Vector3d(0.0, -kDy, 0.0));
//   p_l.push_back(Eigen::Vector3d(0.0, kDy, 0.0));
//   p_l.push_back(Eigen::Vector3d(0.0, 0.0, -kDz));
//   p_l.push_back(Eigen::Vector3d(0.0, 0.0, kDz));
//   std::vector<Eigen::Vector3d> hyperplane_list;
//   std::vector<Eigen::Vector3d> tangent_point_list;
//   for (int i = 0; i < 6; ++i) {
//     Eigen::Vector3d a_l;
//     a_l = u_l[i] / (u_l[i].dot(p_l[i]));
//     tangent_point_list.push_back(p_l[i]);
//     hyperplane_list.push_back(a_l);
//   }

//   // Keep points inside the local box only
//   pcl::PointCloud<pcl::PointXYZ>* pcl_in_box(
//       new pcl::PointCloud<pcl::PointXYZ>());
//   for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
//     // Check all 6 hyperplanes
//     const double kDSign = 0.05;  // numeric issue
//     double sign;
//     int i = 0;
//     for (i = 0; i < 6; ++i) {
//       sign = p->x * hyperplane_list[i].x() + p->y * hyperplane_list[i].y() +
//              p->z * hyperplane_list[i].z() - 1;
//       if (sign > kDSign) break;
//     }
//     if (i == 6) {
//       // inside the local box
//       pcl_in_box->push_back(*p);
//     }
//   }
//   if (pcl_in_box->size())
//     pcl::copyPointCloud(*pcl_in_box, *pcl_tf);
//   else {
//     // full free space --> keep current vertex.
//     return true;
//   }

//   // Find closest point
//   double dist_min_sq = std::numeric_limits<double>::max();
//   Eigen::Vector3d p_tangent;
//   for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
//     double dist_t = p->x * p->x + p->y * p->y + p->z * p->z;
//     if (dist_t < dist_min_sq) {
//       dist_min_sq = dist_t;
//       p_tangent << p->x, p->y, p->z;
//     }
//   }

//   const double kDDist = 0.01;  // deal with numeric error.
//   if ((dist_min_sq == std::numeric_limits<double>::max()) ||
//       (dist_min_sq < kDDist)) {
//     // the path is too close to obstacle.
//     return false;
//   }

//   double a = radius, b = radius, c = radius;  // dimensions of the ellipsoid.
//   // Check if we need to adjust the sphere to ellipsoid.
//   if (dist_min_sq < (radius * radius)) {
//     // Reduce other axes
//     b = std::sqrt(
//         (p_tangent.y() * p_tangent.y() + p_tangent.z() * p_tangent.z()) /
//         (1 - p_tangent.x() * p_tangent.x() / (a * a)));
//     c = b;  // Set equal b for now; but could increase.???
//     // Fit the first hyperplane: x x_l + y y_l + z z_l = 1
//     Eigen::Vector3d hyperplane_last =
//         Eigen::Vector3d(p_tangent.x() / (a * a), p_tangent.y() / (b * b),
//                         p_tangent.z() / (c * c));
//     hyperplane_list.push_back(hyperplane_last);
//     tangent_point_list.push_back(p_tangent);
//   }

//   // Increase the ellipsoid and repeat.
//   bool stop = false;
//   int n_max = 0;  // magic number: max 50 hyperplanes
//   while ((!stop) && (n_max < 50)) {
//     ++n_max;
//     pcl::PointCloud<pcl::PointXYZ>* pcl_reduced(
//         new pcl::PointCloud<pcl::PointXYZ>());
//     // Also re-scale each dimension followed the dimentions of ellipsoid
//     if (hyperplane_list.size()) {
//       Eigen::Vector3d hyperplane_last;
//       hyperplane_last = hyperplane_list.back();
//       // Reduce point: keep points on the same side with zero origin (sign < 0)
//       for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
//         double sign = p->x * hyperplane_last.x() + p->y * hyperplane_last.y() +
//                       p->z * hyperplane_last.z() - 1;
//         const double kDSign = -0.05;  // numeric issue
//         if (sign < kDSign) {
//           // same side with the ellipsoid.
//           pcl_reduced->push_back(*p);
//         }
//       }
//     } else {
//       pcl::copyPointCloud(*pcl_tf, *pcl_reduced);
//     }

//     Eigen::Vector3d p_tangent1;
//     dist_min_sq = std::numeric_limits<double>::max();
//     for (auto p = pcl_reduced->begin(); p != pcl_reduced->end(); ++p) {
//       // Scale to get next closest point.
//       pcl::PointXYZ pv;
//       pv.x = p->x / a;
//       pv.y = p->y / b;
//       pv.z = p->z / c;
//       double dist_t = pv.x * pv.x + pv.y * pv.y + pv.z * pv.z;
//       if (dist_t < dist_min_sq) {
//         dist_min_sq = dist_t;
//         p_tangent1 << p->x, p->y, p->z;
//       }
//     }
//     if ((pcl_reduced->size() == 0) ||
//         (dist_min_sq == std::numeric_limits<double>::max())) {
//       stop = true;
//     } else {
//       double e_ext = dist_min_sq;
//       Eigen::Vector3d hyperplane_new = Eigen::Vector3d(
//           p_tangent1.x() / (a * a * e_ext), p_tangent1.y() / (b * b * e_ext),
//           p_tangent1.z() / (c * c * e_ext));
//       hyperplane_list.push_back(hyperplane_new);
//       tangent_point_list.push_back(p_tangent1);
//       pcl_tf->clear();
//       pcl::copyPointCloud(*pcl_reduced, *pcl_tf);
//     }
//   }
//   if (!stop) {
//     // Require too many hyperplanes
//     return false;
//   }

//   // Find the polygon formed from intersections between the bisector plane vs.
//   // all hyperplanes, Not sure how to get the closed-form solution, also issue
//   // with unknown voxels
//   //  --> get average from uniform sampling on the y-z plane (body coordinate)
//   std::vector<Eigen::Vector3d> feasible_samples;
//   for (double dy = -kDy; dy < kDy; dy += 0.1) {
//     for (double dz = -kDz; dz < kDz; dz += 0.1) {
//       Eigen::Vector3d p(radius, dy, dz);
//       // check if this is inside all hyperplanes.
//       const double kDSign = -0.05;  // numeric issue
//       double sign;
//       int i = 0;
//       for (i = 0; i < hyperplane_list.size(); ++i) {
//         sign = p.x() * hyperplane_list[i].x() + p.y() * hyperplane_list[i].y() +
//                p.z() * hyperplane_list[i].z() - 1;
//         if (sign > kDSign) break;
//       }
//       if (i == hyperplane_list.size()) {
//         ROS_WARN("feasible sample");
//         feasible_samples.push_back(p);
//       }
//     }
//   }

//   for (int i = 0; i < hyperplane_list.size(); ++i) {
//     tangent_point_list[i] =
//         tf_W2S * tangent_point_list[i];  // convert back to world
//     Eigen::Matrix4d tf_inv_T = tf_W2S.matrix().inverse().transpose();
//     Eigen::Vector4d v_t;
//     v_t = tf_inv_T * Eigen::Vector4d(hyperplane_list[i].x(),
//                                      hyperplane_list[i].y(),
//                                      hyperplane_list[i].z(), -1.0);
//     v_t = v_t / (-v_t[3]);
//     hyperplane_list[i] << v_t.x(), v_t.y(), v_t.z();
//   }

//   p1_mod << 0.0, 0.0, 0.0;
//   int feasible_count = 0;
//   for (int i = 0; i < feasible_samples.size(); ++i) {
//     feasible_samples[i] =
//         tf_W2S * feasible_samples[i];  // convert back to world
//     // check if this is free voxel to deal with occluded area.
//     if (map_manager_->getVoxelStatus(feasible_samples[i]) ==
//         MapManager::VoxelStatus::kFree) {

//       p1_mod = p1_mod + feasible_samples[i];
//       ++feasible_count;
//     }
//   }

//   if (feasible_count) {
//     p1_mod = p1_mod / feasible_count;
//   } else {
//     return false;
//   }

//   visualization_[active_id_]->visualizeHyperplanes(p_center, hyperplane_list,
//                                        tangent_point_list);
//   return true;
// }

} // prm
} // search