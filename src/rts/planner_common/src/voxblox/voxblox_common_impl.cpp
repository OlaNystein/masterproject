// #include "planner_common/map_manager_voxblox_impl.h"

// // namespace explorer {

// // TSDF
// template<typename SDFServerType, typename SDFVoxelType>
// voxblox::Layer<SDFVoxelType>* MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSDFLayer() {
//   return sdf_server_.getTsdfMapPtr()->getTsdfLayerPtr();
// }

// // ESDF
// template<>
// voxblox::Layer<voxblox::EsdfVoxel>* MapManagerVoxblox<voxblox::EsdfServer,voxblox::EsdfVoxel>::getSDFLayer() {
//   return sdf_server_.getEsdfMapPtr()->getEsdfLayerPtr();
// }

// template<typename SDFServerType, typename SDFVoxelType>
// MapManagerVoxblox<SDFServerType, SDFVoxelType>::MapManagerVoxblox(const ros::NodeHandle& nh,
//                                                                   const ros::NodeHandle& nh_private)
//     : MapManager(nh, nh_private)
//     , sdf_server_(nh, nh_private)
//     , occupancy_distance_voxelsize_factor_(1.0F)
// {
//   sdf_layer_ = getSDFLayer();
//   CHECK_NOTNULL(sdf_layer_);

//   //Get local parameters from passed nodehandle
//   if (!nh_private.getParam("occupancy_distance_voxelsize_factor", occupancy_distance_voxelsize_factor_)) {
//     ROS_INFO_STREAM("MapManagerVoxblox: failed to find parameter for occupancy_distance_voxelsize_factor, using default of: " << occupancy_distance_voxelsize_factor_);
//   }

//   //Setup E/TsdfIntegratorBase::Config amd object separately (also called in e/tsdf_server_ ctor)
//   tsdf_integrator_config_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private);
//   esdf_integrator_config_ = voxblox::getEsdfIntegratorConfigFromRosParam(nh_private);

//   #if (COL_CHECK_METHOD==0)
//   ROS_INFO("[MapManager]: Point collision checking method: Box check");
//   #elif (COL_CHECK_METHOD==1)
//   ROS_INFO("[MapManager]: Point collision checking method: Direct T/ESDF");
//   #endif

//   #if (EDGE_CHECK_METHOD==0)
//   ROS_INFO("[MapManager]: Line collision checking method: Multiple box checks");
//   #elif (EDGE_CHECK_METHOD==1)
//   ROS_INFO("[MapManager]: Line collision checking method: Cuboid around the");
//   #elif (EDGE_CHECK_METHOD==2)
//   ROS_INFO("[MapManager]: Line collision checking method: Direct T/ESDF check");
//   #endif

//   #if (RAY_CAST_METHOD==0)
//   ROS_INFO("[MapManager]: Ray casting method: Original");
//   #elif (RAY_CAST_METHOD==1)
//   ROS_INFO("[MapManager]: Ray casting method: Iterative");
//   #endif

//   // local_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obb_local_pcl", 10);
// }

// template<typename SDFServerType, typename SDFVoxelType>
// double MapManagerVoxblox<SDFServerType, SDFVoxelType>::getResolution() const {
//   return sdf_layer_->voxel_size();
// }

// template<typename SDFServerType, typename SDFVoxelType>
// bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::getStatus() const { return true; }

// // TSDF
// template<typename SDFServerType, typename SDFVoxelType>
// bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::checkUnknownStatus(const SDFVoxelType* voxel) const {
//   if (voxel == nullptr || voxel->weight < 1e-6) {
//     return true;
//   }
//   return false;
// }

// //ESDF
// template<>
// bool MapManagerVoxblox<voxblox::EsdfServer,voxblox::EsdfVoxel>::checkUnknownStatus(const voxblox::EsdfVoxel* voxel) const {
//   if (voxel == nullptr || !voxel->observed) {
//     return true;
//   }
//   return false;
// }

// template<typename SDFServerType, typename SDFVoxelType>
// typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
//   MapManagerVoxblox<SDFServerType, SDFVoxelType>::getVoxelStatus(const Eigen::Vector3d& position) const {
//   SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByCoordinates(
//       position.cast<voxblox::FloatingPoint>());

//   if (checkUnknownStatus(voxel)) {
//     return VoxelStatus::kUnknown;
//   }
//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
//   if (voxel->distance <= distance_thres) {
//     return VoxelStatus::kOccupied;
//   }
//   return VoxelStatus::kFree;
// }

// template<typename SDFServerType, typename SDFVoxelType>
// typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
//   MapManagerVoxblox<SDFServerType, SDFVoxelType>::getVoxelStatusWithSurfaceStatus(const Eigen::Vector3d& position,
//                                                                                   int& seen) const {
//   SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByCoordinates(
//       position.cast<voxblox::FloatingPoint>());

//   seen = -1;
//   if (checkUnknownStatus(voxel)) {
//     return VoxelStatus::kUnknown;
//   }
//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
//   if (voxel->distance <= distance_thres) {
//     if(voxel->color.r == 255)
//       seen = 0;
//     else if(voxel->color.g == 1)
//       seen = 1;
//     return VoxelStatus::kOccupied;
//   }
//   return VoxelStatus::kFree;
// }

// template<typename SDFServerType, typename SDFVoxelType>
// typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
//   MapManagerVoxblox<SDFServerType, SDFVoxelType>::getRayStatus(const Eigen::Vector3d& view_point,
//                                                                const Eigen::Vector3d& voxel_to_test,
//                                                                bool stop_at_unknown_voxel) const {
//   // This involves doing a raycast from view point to voxel to test.
//   // Let's get the global voxel coordinates of both.
//   float voxel_size = sdf_layer_->voxel_size();
//   float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
//   const voxblox::Point end_scaled =
//       voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   voxblox::LongIndexVector global_voxel_indices;
//   voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
//   // Iterate over the ray.
//   for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
//     SDFVoxelType* voxel =
//         sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//     if (checkUnknownStatus(voxel)) {
//       if (stop_at_unknown_voxel) {
//         return VoxelStatus::kUnknown;
//       }
//     } else if (voxel->distance <= distance_thres) {
//       return VoxelStatus::kOccupied;
//     }
//   }
//   return VoxelStatus::kFree;
// }


// template<typename SDFServerType, typename SDFVoxelType>
// typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
//   MapManagerVoxblox<SDFServerType, SDFVoxelType>::getRayStatus(const Eigen::Vector3d& view_point,
//                                                                const Eigen::Vector3d& voxel_to_test,
//                                                                bool stop_at_unknown_voxel,
//                                                                Eigen::Vector3d& end_voxel) const {
//   // This involves doing a raycast from view point to voxel to test.
//   // Let's get the global voxel coordinates of both.
//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   // const voxblox::Point start_scaled =
//   //     view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;
//   // const voxblox::Point end_scaled =
//   //     voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   voxblox::LongIndexVector global_voxel_indices;
//   // voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
//   // Iterate over the ray.
//   // ROS_INFO("Global indices size: %d", global_voxel_indices.size());

//   Eigen::Vector3d ray = (voxel_to_test - view_point);
//   double ray_len = ray.norm();
//   // ROS_INFO("Ray length: %f", ray_len);
//   Eigen::Vector3d ray_normed = ray / ray_len;
//   // double step_size = ray_len * voxel_size_inv;
//   for (double d=0; d<ray_len; d+=voxel_size*0.9) {
//     Eigen::Vector3d voxel_coordi = view_point + d * ray_normed;
//     // std::cout << "voxel_coordi: " << std::endl;
//     // std::cout << voxel_coordi << std::endl;
//     voxblox::LongIndex center_voxel_index = voxblox::getGridIndexFromPoint<voxblox::LongIndex>(voxel_coordi.cast<voxblox::FloatingPoint>(), voxel_size_inv);
//     SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
//     if (checkUnknownStatus(voxel)) {
//       if (stop_at_unknown_voxel) {
//         end_voxel = voxel_coordi;
//         return VoxelStatus::kUnknown;
//         // ROS_INFO("GRS: Stopping at unknown");
//       }
//     } else if (voxel->distance <= distance_thres) {
//       end_voxel = voxel_coordi;
//       // ROS_INFO("GRS: Stopping at occupied");
//       return VoxelStatus::kOccupied;
//     }
//   }

//   // int k=0;
//   // for (k=0; k<global_voxel_indices.size(); ++k) {
//   //   const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//   //   SDFVoxelType* voxel =
//   //       sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//   //   std::cout << "Voxel index: " << global_index << std::endl;
//   //   ROS_INFO("Voxel distance: %f", voxel->distance);
//   //   if (checkUnknownStatus(voxel)) {
//   //     if (stop_at_unknown_voxel) {
//   //       end_voxel = 
//   //         voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//   //       return VoxelStatus::kUnknown;
//   //       ROS_INFO("GRS: Stopping at unknown");
//   //     }
//   //   } else if (voxel->distance <= distance_thres) {
//   //     end_voxel = 
//   //       voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//   //       ROS_INFO("GRS: Stopping at occupied");
//   //     return VoxelStatus::kOccupied;
//   //   }
//   // }
//   // ROS_INFO("k: %d", k);
//   // ROS_INFO("GRS: Full traversal");
//   end_voxel = voxel_to_test;
//   // end_voxel = view_point;
//   return VoxelStatus::kFree;
// }


// template<typename SDFServerType, typename SDFVoxelType>
// typename MapManagerVoxblox<SDFServerType, SDFVoxelType>::VoxelStatus
//   MapManagerVoxblox<SDFServerType, SDFVoxelType>::getBoxStatusInVoxels(const voxblox::LongIndex& box_center,
//                                                                        const voxblox::AnyIndex& box_voxels,
//                                                                        bool stop_at_unknown_voxel) const {
//   VoxelStatus current_status = VoxelStatus::kFree;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;
//   voxblox::LongIndex voxel_index = box_center;
//   for (voxel_index.x() = box_center.x() - box_voxels.x() / 2;
//        voxel_index.x() < box_center.x() + std::ceil(box_voxels.x() / 2.0);
//        voxel_index.x()++) {
//     for (voxel_index.y() =
//              box_center.y() - box_voxels.y() / 2;
//          voxel_index.y() <
//          box_center.y() + std::ceil(box_voxels.y() / 2.0);
//          voxel_index.y()++) {
//       for (voxel_index.z() =
//                box_center.z() - box_voxels.z() / 2;
//            voxel_index.z() <
//            box_center.z() + std::ceil(box_voxels.z() / 2.0);
//            voxel_index.z()++) {
//         SDFVoxelType* voxel =
//             sdf_layer_->getVoxelPtrByGlobalIndex(voxel_index);
//         if (checkUnknownStatus(voxel)) {
//           if (stop_at_unknown_voxel) {
//             return VoxelStatus::kUnknown;
//           }
//           current_status = VoxelStatus::kUnknown;
//         } else if (voxel->distance <= distance_thres) {
//           return VoxelStatus::kOccupied;
//         }
//       }
//     }
//   }
//   return current_status;
// }

// // [NEED TO REVIEW]
// template<typename SDFServerType, typename SDFVoxelType>
// float MapManagerVoxblox<SDFServerType, SDFVoxelType>::getVoxelDistance(const Eigen::Vector3d& center) const
// {
//   float voxel_size = sdf_layer_->voxel_size();
//   float voxel_size_inv = 1.0 / voxel_size;  // Get the center of the bounding box as a global index.
//   voxblox::LongIndex center_voxel_index = voxblox::getGridIndexFromPoint<voxblox::LongIndex>(center.cast<voxblox::FloatingPoint>(), voxel_size_inv);
//   SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(center_voxel_index);
//   if(checkUnknownStatus(voxel))
//     return -1.0;
//   return voxel->distance;
// }


// // TSDF
// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::clearIfUnknown(SDFVoxelType& voxel) {
//   static constexpr float visualizeDistanceIntensityTsdfVoxels_kMinWeight = 1e-3 + 1e-6;  //value for points to appear with visualizeDistanceIntensityTsdfVoxels
//   if (voxel.weight < 1e-6) {
//     voxel.weight = visualizeDistanceIntensityTsdfVoxels_kMinWeight;
//     voxel.distance = tsdf_integrator_config_.default_truncation_distance;
//   }
// }

// //ESDF
// template<>
// void MapManagerVoxblox<voxblox::EsdfServer,voxblox::EsdfVoxel>::clearIfUnknown(voxblox::EsdfVoxel& voxel) {
//   if (!voxel.observed) {
//     voxel.observed = true;
//     voxel.hallucinated = true;
//     voxel.distance = esdf_integrator_config_.default_distance_m;
//   }
// }

// template<typename SDFServerType, typename SDFVoxelType>
// bool MapManagerVoxblox<SDFServerType, SDFVoxelType>::augmentFreeBox(const Eigen::Vector3d& position,
//                                                                     const Eigen::Vector3d& box_size) {
//   voxblox::HierarchicalIndexMap block_voxel_list;
//   voxblox::utils::getAndAllocateBoxAroundPoint(position.cast<voxblox::FloatingPoint>(),
//                                                box_size,
//                                                sdf_layer_, &block_voxel_list);
//   for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv : block_voxel_list) {
//     // Get block.
//     typename voxblox::Block<SDFVoxelType>::Ptr block_ptr = sdf_layer_->getBlockPtrByIndex(kv.first);

//     for (const voxblox::VoxelIndex& voxel_index : kv.second) {
//       if (!block_ptr->isValidVoxelIndex(voxel_index)) {
//         continue;
//       }
//       SDFVoxelType& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
//       // Clear voxels that haven't been cleared yet
//       clearIfUnknown(voxel);
//     }
//   }
//   return true;
// }

// // [NEED TO REVIEW]
// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getFreeSpacePointCloud(std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//   pcl::PointCloud<pcl::PointXYZ> free_cloud;
//   // std::vector<Eigen::Vector3d> multiray_endpoints;
//   Eigen::Vector3d state_vec(state[0], state[1], state[2]);
//   // StateVec state_ip;
//   // state_ip[0] = state[0];
//   // state_ip[1] = state[1];
//   // state_ip[2] = state[2];
//   // state_ip[3] = state[3];
//   // sensor_params_.sensor[sensor_name].getFrustumEndpoints(state_ip, multiray_endpoints);
//   // std::cout << "Got eps" << std::endl;
//   for(auto ep:multiray_endpoints)
//   {
//     Eigen::Vector3d ray = (ep - state_vec);
//     double ray_length = ray.norm();
//     double voxel_size = 0.1;
//     bool hit = false;
//     // std::cout << "Ray calculated" << std::endl;
//     for(int i=0;i<(int)(ray_length/voxel_size);i++)
//     {
//       Eigen::Vector3d p = i*voxel_size*ray + state_vec;
//       VoxelStatus voxel_state = getVoxelStatus(p);
//       // std::cout << "Got voxel status" << std::endl;
//       if(voxel_state == VoxelStatus::kOccupied)
//       {
//         // std::cout << "Hit" << std::endl;
//         hit = true;
//         break;
//       }
//     }
//     if(!hit)
//     {
//       // std::cout << "Not hit" << std::endl;
//       pcl::PointXYZ data;
//       Eigen::Matrix3d rot_W2B;
//       rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
//                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
//                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
//       Eigen::Matrix3d rot_B2W;
//       rot_B2W = rot_W2B.inverse();
//       Eigen::Vector3d origin(state[0], state[1], state[2]);
//       Eigen::Vector3d epb = rot_B2W*(ep - origin);
//       data.x = epb(0);
//       data.y = epb(1);
//       data.z = epb(2);
//       // data.x = ep(0);
//       // data.y = ep(1);
//       // data.z = ep(2);
//       // std::cout << "current size" << cloud->points.size() << std::endl;
//       cloud->points.push_back(data);
//       // std::cout << "Point added" << std::endl;
//     }
//   }
//   // std::cout << "Setting cloud" << std::endl;
//   // cloud = free_cloud;
//   // std::cout << "Cloud set" << std::endl;
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getScanStatusWithSurface(Eigen::Vector3d& pos,
//                                                                    std::vector<Eigen::Vector3d>& multiray_endpoints,
//                                                                    Eigen::Vector3d bound_min, Eigen::Vector3d bound_max,
//                                                                    std::tuple<int, int, int, int> & gain_log,
//                                                                    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log) {
//   unsigned int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0, num_unknown_surf_voxels = 0;

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   //GBPLANNER_NOTES: no optimization / no twice-counting considerations possible without refactoring planning strategy here

//   // Iterate for every endpoint, insert unknown voxels found over every ray into a set to avoid double-counting
//     // Important: do not use <VoxelIndex> type directly, will break voxblox's implementations
//   // Move away from std::unordered_set and work with std::vector + std::unique count at the end (works best for now)
//   /*std::vector<std::size_t> raycast_unknown_vec_, raycast_occupied_vec_, raycast_free_vec_;
//   raycast_unknown_vec_.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays
//   raycast_free_vec_.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays
//   raycast_occupied_vec_.reserve(multiray_endpoints.size()); //optimize for number of rays*/
//   voxel_log.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays
//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     const voxblox::Point end_scaled =
//       multiray_endpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       // Check if within bounds
//       Eigen::Vector3d voxel_coordi;
//       voxel_coordi = voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//       int j=0;
//       for (j = 0; j < 3; j++) {
//         if ((voxel_coordi[j] < bound_min[j]) || (voxel_coordi[j] > bound_max[j])) break;
//       }
//       if(j<3)
//         continue;

//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown      
//       if (checkUnknownStatus(voxel)) {
//         /*raycast_unknown_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
//         ++num_unknown_voxels;
//         voxel_log.push_back(std::make_pair(voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>(), VoxelStatus::kUnknown));
//         continue;
//       }
//       // Free
//       if (voxel->distance > distance_thres) {
//         /*raycast_free_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
//         ++num_free_voxels;
//         voxel_log.push_back(std::make_pair(voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>(), VoxelStatus::kFree));
//         continue;
//       }
//       // Occupied
//       /*raycast_occupied_vec_.push_back(std::hash<voxblox::GlobalIndex>()(global_index));*/
//       ++num_occupied_voxels;
//       ++num_unknown_surf_voxels;
//       voxel_log.push_back(std::make_pair(voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>(), VoxelStatus::kOccupied));
//       break;
//     }
//   }
//   /*std::sort(raycast_unknown_vec_.begin(), raycast_unknown_vec_.end());
//   std::sort(raycast_occupied_vec_.begin(), raycast_occupied_vec_.end());
//   std::sort(raycast_free_vec_.begin(), raycast_free_vec_.end());
//   num_unknown_voxels = std::unique(raycast_unknown_vec_.begin(), raycast_unknown_vec_.end()) - raycast_unknown_vec_.begin();
//   num_occupied_voxels = std::unique(raycast_occupied_vec_.begin(), raycast_occupied_vec_.end()) - raycast_occupied_vec_.begin();
//   num_free_voxels = std::unique(raycast_free_vec_.begin(), raycast_free_vec_.end()) - raycast_free_vec_.begin();*/
//   gain_log = std::make_tuple(num_unknown_voxels, num_free_voxels, num_occupied_voxels, num_unknown_surf_voxels);
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::setSurfaceColor(Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints, 
//                                                                     int colour) {

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     const voxblox::Point end_scaled =
//       multiray_endpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         if(colour == 0)  // red
//           voxel->color.r = 255;  // Setting the color of the occupied (surface) voxel
//         else if(colour == 1)  // green
//           voxel->color.g = 1;
//         break;
//       }
//     }
//   }
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::setSurfaceColor(Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints, 
//                                                                     int colour, 
//                                                                     float darkness_range) {
// // This is probably not working
//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     auto endpoint = multiray_endpoints[i] * darkness_range;
//     const voxblox::Point end_scaled = endpoint.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         if(colour == 0)  // red
//           voxel->color.r = 255;  // Setting the color of the occupied (surface) voxel
//         else if(colour == 1)  // green
//           voxel->color.g = 1;
//         break;
//       }
//     }
//   }
// }


// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSurfaceGain(Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints,
//                                                                     int & num_unknown_surf_voxels) {

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   num_unknown_surf_voxels = 0;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     const voxblox::Point end_scaled =
//       multiray_endpoints[i].cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         if(voxel->color.r < 255) {
//           num_unknown_surf_voxels++;
//         }
//         break;
//       }
//     }
//   }
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSurfaceGain(Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints,
//                                                                     int & num_unknown_surf_voxels,
//                                                                     std::map<int, int>& surf_voxels_angle_map,
//                                                                     double darkness_range) {

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   num_unknown_surf_voxels = 0;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   std::vector<std::size_t> raycast_unseen_vec_;
//   raycast_unseen_vec_.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     auto endpoint = multiray_endpoints[i] * darkness_range;
//     const voxblox::Point end_scaled =
//       endpoint.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         if(voxel->color.r < 255) {
//           num_unknown_surf_voxels++;
//           Eigen::Vector3d ray;
//           ray = (endpoint - pos);
//           double theta = atan2(ray(1), ray(0));
//           if (theta > M_PI)
//             theta -= 2 * M_PI;
//           else if (theta < -M_PI)
//             theta += 2 * M_PI;
//           int theta_int = (int)(theta*180/M_PI + 0.4999);
//           surf_voxels_angle_map[theta_int] += 1;
//         }
//         break;
//       }
//     }
//   }
//   // ROS_INFO("Num unseen surf voxels: %d", num_unknown_surf_voxels);
// }


// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSurfaceGain(const Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints,
//                                                                     const Eigen::Vector3d& bound_min, const Eigen::Vector3d& bound_max,
//                                                                     std::vector<std::vector<std::size_t>>& surf_voxels_angle_map,
//                                                                     double darkness_range) {
//   // std::cout << "GET SURFACE GAIN" << std::endl;
//   // std::cout << "darkness_range: " << darkness_range << " ep size: " << multiray_endpoints.size() << std::endl;

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   // for(int i=-180;i<180;i++) {
//   //   surf_voxels_angle_map[i].clear();
//   // }

//   std::vector<std::size_t> raycast_unseen_vec_;
//   raycast_unseen_vec_.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     auto endpoint = multiray_endpoints[i] * darkness_range;
//     const voxblox::Point end_scaled =
//       endpoint.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     // std::cout << "Next ray:" << std::endl;
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       // Check if the voxel is within bounds
//       Eigen::Vector3d voxel_coordi;
//       voxel_coordi = voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//       int j=0;
//       for (j = 0; j < 3; j++) {
//         if ((voxel_coordi[j] < bound_min[j]) || (voxel_coordi[j] > bound_max[j])) break;
//       }
//       if(j<3) {
//         // ROS_WARN("MM: Voxel out of bounds");
//         continue;
//       }
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);

//       Eigen::Vector3d ray;
//       ray = (multiray_endpoints[i] - pos);
//       // ray = (voxel_coordi - pos);
//       double theta = atan2(ray(1), ray(0));
//       // if (x > M_PI)
//       //   x -= 2 * M_PI;
//       // else if (x < -M_PI)
//       //   x += 2 * M_PI;
//       if (theta > M_PI)
//         theta -= 2 * M_PI;
//       else if (theta < -M_PI)
//         theta += 2 * M_PI;
//       int theta_int = (int)(theta*180.0/M_PI + 0.4999);
//       // if(theta > -1.0 && theta < 1.0)
//           // std::cout << "(" << ray(0) << "," << ray(1) << "): " << theta << " " << theta_int << " " << (int)(voxel->color.r) << std::endl;
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         // if(theta > -0.5 && theta < 0.5)
//           // std::cout << "Unknown: " << theta << " " << theta_int << " " << (int)(voxel->color.r) << std::endl;
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         // if(theta > -1.0 && theta < 1.0)
//         //   std::cout << "Occupied: " << theta << " " << theta_int << " " << (int)(voxel->color.r) << std::endl;
//         if(voxel->color.r < 255) {
//           surf_voxels_angle_map[theta_int + 179].push_back(std::hash<voxblox::GlobalIndex>()(global_index));
//         }
//         break;
//       }
//     }
//   }
//   // ROS_INFO("Num unseen surf voxels: %d", num_unknown_surf_voxels);
// }


// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSurfaceGain(Eigen::Vector3d& pos,
//                                                                     std::vector<Eigen::Vector3d>& multiray_endpoints,
//                                                                     std::map<int, std::vector<std::size_t>>& surf_voxels_angle_map,
//                                                                     BoundedSpaceParams &global_space,
//                                                                     double darkness_range) {

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;

//   const voxblox::Point start_scaled =
//       pos.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   std::vector<std::size_t> raycast_unseen_vec_;
//   raycast_unseen_vec_.reserve(multiray_endpoints.size() * tsdf_integrator_config_.max_ray_length_m * voxel_size_inv); //optimize for number of rays

//   for (size_t i=0; i<multiray_endpoints.size(); ++i) {
//     auto endpoint = multiray_endpoints[i] * darkness_range;
//     const voxblox::Point end_scaled =
//       endpoint.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//     voxblox::LongIndexVector global_voxel_indices;
//     voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//     // Iterate over the ray.
//     for (size_t k=0; k<global_voxel_indices.size(); ++k) {
//       const voxblox::GlobalIndex& global_index = global_voxel_indices[k];
//       // Check if the voxel is within bounds
//       Eigen::Vector3d voxel_coordi;
//       voxel_coordi = voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//       SDFVoxelType* voxel =
//           sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//       // Unknown
//       if (checkUnknownStatus(voxel)) {
//         continue;
//       }
//       // Occupied
//       if (voxel->distance < distance_thres) {
//         if(voxel->color.r < 255) {
//           if(global_space.isInsideSpace(voxel_coordi)) {
//             Eigen::Vector3d ray;
//             ray = (endpoint - pos);
//             double theta = atan2(ray(1), ray(0));
//             if (theta > M_PI)
//               theta -= 2 * M_PI;
//             else if (theta < -M_PI)
//               theta += 2 * M_PI;
//             int theta_int = (int)(theta*180/M_PI);
//             surf_voxels_angle_map[theta_int].push_back(std::hash<voxblox::GlobalIndex>()(global_index));
//           }
//         }
//         break;
//       }
//     }
//   }
//   // ROS_INFO("Num unseen surf voxels: %d", num_unknown_surf_voxels);
// }


// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::augmentFreeFrustum() {
//   ROS_WARN_THROTTLE(5.0, "MapManagerVoxblox::augmentFreeFrustum: N/A");
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalMap(const Eigen::Vector3d& center,
//                                                                      const Eigen::Vector3d& bounding_box_size,
//                                                                      std::vector<Eigen::Vector3d>& occupied_voxels,
//                                                                      std::vector<Eigen::Vector3d>& free_voxels) {
//   // ROS_WARN_THROTTLE(5.0,
//   //                   "MapManagerVoxblox::extractLocalMap --> Temporary solution "
//   //                   "to be consistent with Octomap interface.");
//   occupied_voxels.clear();
//   free_voxels.clear();
//   double resolution = getResolution();

//   int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) * 2;  // always even number
//   int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
//   int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

//   // Correct center voxel depeding on the resolution of the map.
//   const Eigen::Vector3d center_corrected(
//       resolution * std::floor(center.x() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.y() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.z() / resolution) + resolution / 2.0);

//   Eigen::Vector3d origin_offset;
//   origin_offset = center_corrected - Eigen::Vector3d(Nx * resolution / 2,
//                                                      Ny * resolution / 2,
//                                                      Nz * resolution / 2);
//   for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
//     for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
//       for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
//         Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
//                             z_ind * resolution);
//         pos = pos + origin_offset;
//         VoxelStatus vs = getVoxelStatus(pos);
//         if (vs == VoxelStatus::kFree) {
//           free_voxels.push_back(pos);
//         } else if (vs == VoxelStatus::kOccupied) {
//           occupied_voxels.push_back(pos);
//         }
//       }
//     }
//   }
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalMapAlongAxis(const Eigen::Vector3d& center,
//                                                                               const Eigen::Vector3d& axis,
//                                                                               const Eigen::Vector3d& bounding_box_size,
//                                                                               std::vector<Eigen::Vector3d>& occupied_voxels,
//                                                                               std::vector<Eigen::Vector3d>& free_voxels) {

//   occupied_voxels.clear();
//   free_voxels.clear();
//   double resolution = getResolution();

//   int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) * 2;  // always even number
//   int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
//   int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

//   // Correct center voxel depeding on the resolution of the map.
//   const Eigen::Vector3d center_corrected(
//       resolution * std::floor(center.x() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.y() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.z() / resolution) + resolution / 2.0);

//   Eigen::Vector3d origin_offset;
//   Eigen::Vector3d half_box (Nx * resolution / 2, Ny * resolution / 2, Nz * resolution / 2);
//   origin_offset = center_corrected - half_box;

//   Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
//   Eigen::Quaternion<double> quat_W2S;
//   quat_W2S.setFromTwoVectors(x_axis, axis.normalized());

//   for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
//     for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
//       for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
//         Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
//                             z_ind * resolution);
//         pos = pos - half_box;
//         pos = quat_W2S * pos + center_corrected;
//         VoxelStatus vs = getVoxelStatus(pos);
//         if (vs == VoxelStatus::kFree) {
//           free_voxels.push_back(pos);
//         } else if (vs == VoxelStatus::kOccupied) {
//           occupied_voxels.push_back(pos);
//         }
//       }
//     }
//   }
// }

// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalSurfaceMap(const Eigen::Vector3d& center,
//                                                                      const Eigen::Vector3d& bounding_box_size,
//                                                                      std::vector<Eigen::Vector3d>& surface_voxels,
//                                                                      std::vector<Eigen::Vector3d>& seen_voxels) {
//   // ROS_WARN_THROTTLE(5.0,
//   //                   "MapManagerVoxblox::extractLocalMap --> Temporary solution "
//   //                   "to be consistent with Octomap interface.");
//   surface_voxels.clear();
//   seen_voxels.clear();
//   double resolution = getResolution();

//   int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) * 2;  // always even number
//   int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
//   int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

//   // Correct center voxel depeding on the resolution of the map.
//   const Eigen::Vector3d center_corrected(
//       resolution * std::floor(center.x() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.y() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.z() / resolution) + resolution / 2.0);

//   Eigen::Vector3d origin_offset;
//   origin_offset = center_corrected - Eigen::Vector3d(Nx * resolution / 2,
//                                                      Ny * resolution / 2,
//                                                      Nz * resolution / 2);
//   for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
//     for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
//       for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
//         Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
//                             z_ind * resolution);
//         pos = pos + origin_offset;
//         int seen;
//         VoxelStatus vs = getVoxelStatusWithSurfaceStatus(pos, seen);
//         if (vs == VoxelStatus::kFree) {
//           continue;
//         } else if (vs == VoxelStatus::kOccupied) {
//           if(seen == 0) { // If voxel is seen by the camera
//             seen_voxels.push_back(pos);
//             surface_voxels.push_back(pos);
//           }
//           if(seen == 1)
//             surface_voxels.push_back(pos);
//         }
//       }
//     }
//   }
// }


// template<typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::extractLocalSurfaceMap(const Eigen::Vector3d& center,
//                                                                      const Eigen::Vector3d& bounding_box_size,
//                                                                      const Eigen::Vector3d& global_min,
//                                                                      const Eigen::Vector3d& global_max,
//                                                                      std::vector<Eigen::Vector3d>& surface_voxels,
//                                                                      std::vector<Eigen::Vector3d>& seen_voxels) {
//   // ROS_WARN_THROTTLE(5.0,
//   //                   "MapManagerVoxblox::extractLocalMap --> Temporary solution "
//   //                   "to be consistent with Octomap interface.");
//   surface_voxels.clear();
//   seen_voxels.clear();
//   double resolution = getResolution();

//   int Nx = std::ceil(bounding_box_size.x() / (2 * resolution)) * 2;  // always even number
//   int Ny = std::ceil(bounding_box_size.y() / (2 * resolution)) * 2;
//   int Nz = std::ceil(bounding_box_size.z() / (2 * resolution)) * 2;

//   // Correct center voxel depeding on the resolution of the map.
//   const Eigen::Vector3d center_corrected(
//       resolution * std::floor(center.x() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.y() / resolution) + resolution / 2.0,
//       resolution * std::floor(center.z() / resolution) + resolution / 2.0);

//   Eigen::Vector3d origin_offset;
//   origin_offset = center_corrected - Eigen::Vector3d(Nx * resolution / 2,
//                                                      Ny * resolution / 2,
//                                                      Nz * resolution / 2);
//   for (double x_ind = 0; x_ind <= Nx; ++x_ind) {
//     for (double y_ind = 0; y_ind <= Ny; ++y_ind) {
//       for (double z_ind = 0; z_ind <= Nz; ++z_ind) {
//         Eigen::Vector3d pos(x_ind * resolution, y_ind * resolution,
//                             z_ind * resolution);
//         pos = pos + origin_offset;
        
//         // Checking bounds
//         bool out_bound = false;
//         for (int j = 0; j < 3; j++) {
//           if (pos[j] < global_min[j]){
//             // pos[j] = global_min[j];
//             out_bound = true;
//             break;
//           }
//           else if(pos[j] > global_max[j]){
//             // pos[j] = global_max[j];
//             out_bound = true;
//             break;
//           }
//         }

//         if (out_bound) {
//           continue;
//         }

//         int seen;
//         VoxelStatus vs = getVoxelStatusWithSurfaceStatus(pos, seen);
//         if (vs == VoxelStatus::kFree) {
//           continue;
//         } else if (vs == VoxelStatus::kOccupied) {
//           if(seen == 0) { // If voxel is seen by the camera
//             seen_voxels.push_back(pos);
//             surface_voxels.push_back(pos);
//           }
//           if(seen == 1)
//             surface_voxels.push_back(pos);
//         }
//       }
//     }
//   }
// }

// template <typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getSurfacePCL(pcl::PointCloud<pcl::PointXYZI> &surf_pcl, double surface_dist)
// {
//   voxblox::createSurfaceDistancePointcloudFromTsdfLayer(*sdf_layer_, surface_dist, &surf_pcl);
// }

// template <typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getLocalPointcloud(const Eigen::Vector3d& center,
//                                                                         const double& range,
//                                                                         const double& yaw,
//                                                                         pcl::PointCloud<pcl::PointXYZI>& pcl,
//                                                                         bool include_unknown_voxels)
// {
//   // use ray cast function
//   // x = range * cos (theta) * sin(phi), y = range * sin(theta) * sin(phi), z = range * cos(phi)
//   // theta - 0 to 2pi, phi - 0 to pi

//   pcl::PointXYZI point;
//   point.intensity = 250;

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;
//   const double angular_res = 2.0*M_PI/180.0;
//       // ((2 * range * range - voxel_size * voxel_size) / (2 * range * range)) * M_PI / 180;

//   // std::cout << "Angular range is: " << angular_res << std::endl;

//   const voxblox::Point start_scaled = center.cast<voxblox::FloatingPoint>() * voxel_size_inv;
//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   for (double theta = 0; theta < 2 * M_PI; theta += angular_res) {
//     for (double phi = 0; phi < M_PI; phi += angular_res) {
//       Eigen::Vector3d end_point;
//       end_point << center.x() + range * cos(theta) * sin(phi), center.y() + range * sin(theta) * sin(phi), center.z() + range * cos(phi);
//       const voxblox::Point end_scaled = end_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//       voxblox::LongIndexVector global_voxel_indices;
//       voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//       // Iterate over the ray
//       size_t k = 0;
//       for (k = 0; k < global_voxel_indices.size(); ++k) {
//         const voxblox::GlobalIndex& global_index = global_voxel_indices[k];

//         SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//         // Unknown
//         if (checkUnknownStatus(voxel)) {
//           if (!include_unknown_voxels)
//             continue;
//         }
//         // Free
//         else if (voxel->distance > distance_thres) {
//           continue;
//         }
//         // Occupied
//         Eigen::Vector3d voxel_coordi;
//         voxel_coordi = voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//         point.x = voxel_coordi.x();
//         point.y = voxel_coordi.y();
//         point.z = voxel_coordi.z();
//         // point.x = end_point.x();
//         // point.y = end_point.y();
//         // point.z = end_point.z();
//         pcl.points.push_back(point);
//         break;
//       }
//       // if(k == global_voxel_indices.size()) {
//       //   point.x = end_point.x();
//       //   point.y = end_point.y();
//       //   point.z = end_point.z();
//       //   pcl.points.push_back(point);
//       // }
//     }
//   }
//   // local_pcl_pub_ = nh
//   // sensor_msgs::PointCloud2 obb_local_pcl;
//   // pcl::toROSMsg(pcl.get(), obb_local_pcl);
//   // local_pcl_pub_.publish(obb_local_pcl);

// }

// template <typename SDFServerType, typename SDFVoxelType>
// void MapManagerVoxblox<SDFServerType, SDFVoxelType>::getLocalPointcloud(const Eigen::Vector3d& center,
//                                                                         const double& range,
//                                                                         const double& yaw,
//                                                                         const Eigen::Vector2d& z_limits,
//                                                                         pcl::PointCloud<pcl::PointXYZI>& pcl,
//                                                                         bool include_unknown_voxels)
// {
//   // use ray cast function
//   // x = range * cos (theta) * sin(phi), y = range * sin(theta) * sin(phi), z = range * cos(phi)
//   // theta - 0 to 2pi, phi - 0 to pi

//   pcl::PointXYZI point;
//   point.intensity = 250;

//   const float voxel_size = sdf_layer_->voxel_size();
//   const float voxel_size_inv = 1.0 / voxel_size;
//   const double angular_res = 2.0*M_PI/180.0;
//       // ((2 * range * range - voxel_size * voxel_size) / (2 * range * range)) * M_PI / 180;

//   // std::cout << "Angular range is: " << angular_res << std::endl;

//   const voxblox::Point start_scaled = center.cast<voxblox::FloatingPoint>() * voxel_size_inv;
//   const float distance_thres = occupancy_distance_voxelsize_factor_ * sdf_layer_->voxel_size() + 1e-6;

//   for (double theta = 0; theta < 2 * M_PI; theta += angular_res) {
//     for (double phi = 0; phi < M_PI; phi += angular_res) {
//       Eigen::Vector3d end_point;
//       end_point << center.x() + range * cos(theta) * sin(phi), center.y() + range * sin(theta) * sin(phi), center.z() + range * cos(phi);
//       const voxblox::Point end_scaled = end_point.cast<voxblox::FloatingPoint>() * voxel_size_inv;

//       voxblox::LongIndexVector global_voxel_indices;
//       voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);
//       // Iterate over the ray
//       size_t k = 0;
//       for (k = 0; k < global_voxel_indices.size(); ++k) {
//         const voxblox::GlobalIndex& global_index = global_voxel_indices[k];

//         SDFVoxelType* voxel = sdf_layer_->getVoxelPtrByGlobalIndex(global_index);
//         Eigen::Vector3d voxel_coordi;
//         voxel_coordi = voxblox::getCenterPointFromGridIndex(global_index, voxel_size).cast<double>();
//         if(voxel_coordi(2) > center(2) + z_limits(1) || voxel_coordi(2) < center(2) + z_limits(0)) {
//           break;
//         }
//         // Unknown
//         if (checkUnknownStatus(voxel)) {
//           if (!include_unknown_voxels)
//             continue;
//         }
//         // Free
//         else if (voxel->distance > distance_thres) {
//           continue;
//         }
//         // Occupied
//         point.x = voxel_coordi.x();
//         point.y = voxel_coordi.y();
//         point.z = voxel_coordi.z();
//         // point.x = end_point.x();
//         // point.y = end_point.y();
//         // point.z = end_point.z();
//         pcl.points.push_back(point);
//         break;
//       }
//       // if(k == global_voxel_indices.size()) {
//       //   point.x = end_point.x();
//       //   point.y = end_point.y();
//       //   point.z = end_point.z();
//       //   pcl.points.push_back(point);
//       // }
//     }
//   }
// }

// template <typename SDFServerType, typename SDFVoxelType>
// pcl::PointXYZI MapManagerVoxblox<SDFServerType, SDFVoxelType>::eigenVec3dToPCLPoint(Eigen::Vector3d& vec) const {
//   pcl::PointXYZI point;
//   point.x = vec.x();
//   point.y = vec.y();
//   point.z = vec.z();
//   return point;
// }

// // }  // namespace explorer
