#include "prm/minimap.h"


namespace search{


Minimap::Minimap(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private): nh_(nh), nh_private_(nh_private){

  voxel_sub_ = nh_.subscribe("rimapp_node/occupied_nodes", 10, &Minimap::voxelCallback, this);
  minimap_pub_ = nh_.advertise<sensor_msgs::Image>("rimapp_node/minimap", 10);

  minimap_.header.stamp = ros::Time::now();
  minimap_.encoding = "rgb8";
  minimap_.height = 280;
  minimap_.width = 280;
  minimap_.is_bigendian = false;
  minimap_.step = 3 * minimap_.width;
}

void Minimap::voxelCallback(const visualization_msgs::MarkerArray& msg){
  //ros::Duration(0.5).sleep();
  minimap_.header.stamp = ros::Time::now();
  minimap_.data.clear();

  std::vector<geometry_msgs::Point> points = msg.markers[0].points;
  auto minmaxXIt = std::minmax_element(points.begin(), points.end(), compareX);
  double minX = minmaxXIt.first->x;
  double maxX = minmaxXIt.second->x;


  auto minmaxYIt = std::minmax_element(points.begin(), points.end(), compareY);
  double minY = minmaxYIt.first->y;
  double maxY = minmaxYIt.second->y;

 

  for (int i=0; i <(minimap_.width*minimap_.height); i++){
    minimap_.data.push_back(0);
    minimap_.data.push_back(0);
    minimap_.data.push_back(0);
  }
  

  for (const auto& point : points){
    // flip  and fit axes from world frame to image
    if ((point.z < 0.5) || (point.z > 4)){
      continue;
    }
    int y = fitXToYRange(point.x, maxX, minX);
    int x = fitYToXRange(point.y, maxY, minY);

    if (y < 0) {y = 0;}
    if (x < 0) {x = 0;}
    std::tuple<uint8_t, uint8_t, uint8_t> rgb = fitZtoRGB(point.z);
    minimap_.data[y*minimap_.step + 3*x] = std::get<0>(rgb);
    minimap_.data[y*minimap_.step + 3*x + 1] = std::get<1>(rgb);
    minimap_.data[y*minimap_.step + 3*x + 2] = std::get<2>(rgb);
  }

  for (const auto& state : (*states_for_minimap_)){

    double unit_x = state.second->x();
    double unit_y = state.second->y();

    int y = fitXToYRange(unit_x, maxX, minX);
    int x = fitYToXRange(unit_y, maxY, minY);
 
    if (y < 0) {y = 0;}
    if (x < 0) {x = 0;}
    colorUnit(x, y, state.first);
  }
  for (int i = 0; i < targets_for_minimap_.size(); i++){
    if (targets_for_minimap_[i].first){
      double target_x = targets_for_minimap_[i].second->x();
      double target_y = targets_for_minimap_[i].second->y();
      int y = fitXToYRange(target_x, maxX, minX);
      int x = fitYToXRange(target_y, maxY, minY);
      if (y < 0) {y = 0;}
      if (x < 0) {x = 0;}
      ROS_INFO("Targetminimap: x: %f, y: %f", target_x, target_y);
      colorTarget(x, y, i);
    }
  }



  minimap_pub_.publish(minimap_);
}

void Minimap::colorUnit(int x, int y, int id){
  int x_range = 3;
  int y_range = 3;
  if ((x < x_range)||(x > minimap_.width - x_range)) {
    x_range = x;
  } 
  if ((y < y_range)||(y > minimap_.height - y_range)) {
    y_range = y;
  }
  uint8_t color_r, color_g, color_b;

  switch (id)
  {
  case 0: // red
    color_r = 255;
    color_g = 0;
    color_b = 0;
    break;
  case 1: // orange
    color_r = 255;
    color_g = 128;
    color_b = 0;
    break;
  case 2: // yellow
    color_r = 255;
    color_g = 255;
    color_b = 0;
    break;
  case 3: // green
    color_r = 0;
    color_g = 255;
    color_b = 0;
    break;
  case 4: // pink
    color_r = 255;
    color_g = 51;
    color_b = 153;
    break;
  case 5: // blue
    color_r = 0;
    color_g = 0;
    color_b = 255;
    break;
  case 6: // turqoise
    color_r = 0;
    color_g = 255;
    color_b = 255;
    break;
  default: // dark red, fail
    color_r = 128;
    color_g = 0;
    color_b = 0;
    break;
  }

  for (int x_i = x - x_range; x_i < x + x_range; x_i++){
    for (int y_i = y - y_range; y_i < y + y_range; y_i++){
      minimap_.data[y_i*minimap_.step + 3*x_i] = color_r;
      minimap_.data[y_i*minimap_.step + 3*x_i+1] = color_g;
      minimap_.data[y_i*minimap_.step + 3*x_i+2] = color_b;
    }
  }

  minimap_.data[y*minimap_.step + x] = 255;
  
}

void Minimap::colorTarget(int x, int y, int id){
  int x_range = 3;
  int y_range = 3;
  int x_range_thin = 1;
  int y_range_thin = 1;
  if ((x < x_range)||(x > minimap_.width - x_range)) {
    x_range = x;
  } 
  if ((y < y_range)||(y > minimap_.height - y_range)) {
    y_range = y;
  }
  uint8_t color_r, color_g, color_b;

  switch (id)
  {
  case 0: // red
    color_r = 255;
    color_g = 0;
    color_b = 0;
    break;
  case 1: // orange
    color_r = 255;
    color_g = 128;
    color_b = 0;
    break;
  case 2: // yellow
    color_r = 255;
    color_g = 255;
    color_b = 0;
    break;
  case 3: // green
    color_r = 0;
    color_g = 255;
    color_b = 0;
    break;
  case 4: // pink
    color_r = 255;
    color_g = 51;
    color_b = 153;
    break;
  case 5: // blue
    color_r = 0;
    color_g = 0;
    color_b = 255;
    break;
  case 6: // turqoise
    color_r = 0;
    color_g = 255;
    color_b = 255;
    break;
  default: // dark red, fail
    color_r = 128;
    color_g = 0;
    color_b = 0;
    break;
  }

  for (int x_i = x - x_range_thin; x_i < x + x_range_thin; x_i++){
    for (int y_i = y - y_range; y_i < y + y_range; y_i++){
      minimap_.data[y_i*minimap_.step + 3*x_i] = color_r;
      minimap_.data[y_i*minimap_.step + 3*x_i+1] = color_g;
      minimap_.data[y_i*minimap_.step + 3*x_i+2] = color_b;
    }
  }
  for (int x_i = x - x_range; x_i < x + x_range; x_i++){
    for (int y_i = y - y_range_thin; y_i < y + y_range_thin; y_i++){
      minimap_.data[y_i*minimap_.step + 3*x_i] = color_r;
      minimap_.data[y_i*minimap_.step + 3*x_i+1] = color_g;
      minimap_.data[y_i*minimap_.step + 3*x_i+2] = color_b;
    }
  }

  minimap_.data[y*minimap_.step + x] = 255;
}

bool Minimap::compareX(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return a.x < b.x;
}

bool Minimap::compareY(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return a.y < b.y;
}

int Minimap::fitXToYRange(double x, int XpointMax, int XpointMin) {
  // Return y value in image because of flipped axis
  return (1 - (x - XpointMin)/(XpointMax - XpointMin)) * (minimap_.height);
}
int Minimap::fitYToXRange(double y, int YpointMax, int YpointMin) {
  // Return x value in image because of flipped axis
  return (1 - (y - YpointMin)/(YpointMax - YpointMin)) * (minimap_.width);
}

std::tuple<uint8_t, uint8_t, uint8_t> Minimap::fitZtoRGB(double z) {
  double z_scaled = (z)/3;
  if (z < 0.3){z_scaled = 0;} // remove points close to the ground
  uint8_t red = z_scaled * 255;
  uint8_t green = z_scaled * 255;
  uint8_t blue = z_scaled * 255;
  return std::make_tuple(red, green, blue);
}

void Minimap::setStatePtr(const std::vector<std::pair<int, StateVec*>>& states_for_minimap){
  states_for_minimap_ = &states_for_minimap;
  for (int i = 0; i < states_for_minimap_->size(); i++){
    bool init = false;
    StateVec* target = NULL;
    std::pair<bool, StateVec*> p = std::make_pair(init, target);
    targets_for_minimap_.push_back(p);
  }
}

void Minimap::setTarget(int id, StateVec* state){
  targets_for_minimap_[id].second = state;
}
void Minimap::setTargetStatus(int id, bool active){
  targets_for_minimap_[id].first = active;
}
}