#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "planner/Planner.hpp"
// #include "helpers.h"
// #include "spline.h"

Planner::Planner() {
  dist_inc = 0.3;
  lane = 1;
  ref_vel = 49.5;
}

void Planner::updateCar(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) {
  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_d_ = car_d;
  car_yaw_ = car_yaw;
  car_speed_ = car_speed;
}


// std::vector<double> Planner::planX() {
//     std::vector<double> next_x_vals;
//     for (int i = 0; i < 50; ++i) {


//       double next_s = car_s_ + (i + 1) * dist_inc;
//       double next_d = 6.0;
//       vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);      
      
//       next_x_vals.push_back(xy[0]);
//       // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//     }
//     return next_x_vals;
// }

// std::vector<double> Planner::planY() {
//     std::vector<double> next_y_vals;
//     for (int i = 0; i < 50; ++i) {

//       double next_s = car_s_ + (i + 1) * dist_inc;
//       double next_d = 6.0;
//       vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
//       // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//       next_y_vals.push_back(xy[1]);
//     }
//     return next_y_vals;
// }

void Planner::load_map(std::string filename) {
  std::cout << "Loading map file: " << filename << std::endl;

  std::ifstream in_map_(filename.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
}
