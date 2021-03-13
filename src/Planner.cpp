#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "planner/Planner.hpp"
#include "helpers.h"

enum side {
  LEFT = -1,
  FRONT = 0,
  RIGHT = 1
};

Planner::Planner(std::string map_file) {
  load_map(map_file);
}

void Planner::update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, nlohmann::json previous_path_x, nlohmann::json previous_path_y, double end_path_s) {
  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_d_ = car_d;
  car_yaw_ = car_yaw;
  car_speed_ = car_speed;
  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;
  prev_size = previous_path_x.size();
  if(prev_size > 0) {
    car_s_ = end_path_s;
  }
}

void Planner::track(nlohmann::json sensor_fusion) {
  bool car_front = false;
  bool car_left = false;
  bool car_right = false;

  for(int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];
    double check_car_s = sensor_fusion[i][5];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];

    double check_speed = sqrt(vx*vx + vy+vy);
    check_car_s += ((double)prev_size * 0.02 * check_speed);

    // Check for car in current lane
    if(d < (4 + 4 * current_lane + 0) && d > (4 * current_lane + 0)) {
      auto s_diff = check_car_s - car_s_;
      if((s_diff > - SAFE_DIST_REAR) && (s_diff < SAFE_DIST_FRONT)) {
    
        car_front = true;
      }
    }

    // car_front = proximity(check_car_s, d, FRONT);

    // Check for car in left lane
    if(d < (4 + 4 * (current_lane - 1)) && d > (4 * (current_lane - 1))) {
      auto s_diff = check_car_s - car_s_;
      if ((s_diff > - SAFE_DIST_REAR) && (s_diff < SAFE_DIST_FRONT)) {
    
        car_left = true;
      }
    }

    // car_left = proximity(check_car_s, d, LEFT);

    // Check for car in right lane
    if(d < (4 + 4 * (current_lane + 1)) && d > (4 * (current_lane + 1))) {
      auto s_diff = check_car_s - car_s_;
      if ((s_diff > - SAFE_DIST_REAR) && (s_diff < SAFE_DIST_FRONT)) {
    
        car_right = true;
      }
    }
    // car_right = proximity(check_car_s, d, RIGHT);

  }

  if(car_front) {
    if (!car_left && current_lane > 0) {
      current_lane--;
    } else if (!car_right && current_lane < 2) {
      current_lane++;
    } else {
      velocity -= DELTA_VEL;
    }
      
  } else {
    if((current_lane == 0 && !car_right) || (current_lane == 2 && !car_left)) {
      current_lane = 1;
    } 
    if(velocity < REF_VEL) velocity += DELTA_VEL;
  
  }
}

void Planner::plan(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x_;
  double ref_y = car_y_;
  double ref_yaw = deg2rad(car_yaw_);

  // Determine a fitting starting point for the new path.
  if(prev_size < 2) {
    // use car as starting ref when list is small.
    // tangent points with respect to car.
    double prev_car_x = car_x_ - cos(car_yaw_);
    double prev_car_y = car_y_ - sin(car_yaw_);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x_);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y_);

  } else {
    // when list is big
    // tangent points with respect to prev path last point.

    // redefine ref state to prev path last point.
    ref_x = previous_path_x_[prev_size - 1];
    ref_y = previous_path_y_[prev_size - 1];

    double ref_x_prev = previous_path_x_[prev_size - 2];
    double ref_y_prev = previous_path_y_[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // create points in Frenet, 30m spaced apart., these are anchor points that will be used later
  // to interpolate the points inbetween with the spline library.
  vector<double> next_wp0 = getXY(car_s_ + 30, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s_ + 60, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s_ + 90, (2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // change to car reference frame.
  for(int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw)); 
  }

  // update the spline with the information it needs to interpolate
  spline.set_points(ptsx, ptsy);

  for(int i = 0; i < previous_path_x_.size(); i++) {
    next_x_vals.push_back(previous_path_x_[i]);
    next_y_vals.push_back(previous_path_y_[i]);
  }

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0.0;

  // interpolate points on spline
  for(int i = 1; i <= 50 - previous_path_x_.size(); i++) {
      double N = (target_dist/(0.02*velocity/2.24));
      double x_point = x_add_on + target_x / N;
      double y_point = spline(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;


      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw); 

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
  }
}

bool Planner::proximity(double s, double d, int direction) {
  if(d < (4 + 4 * (current_lane + direction)) && d > (4 * (current_lane + direction))) {
    auto s_diff = s - car_s_;
    if ((s_diff > - SAFE_DIST_REAR) && (s_diff < SAFE_DIST_FRONT)) {
      return true;
    }
  }
  return false;
}

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
  std::cout << "Map waypoints loaded." << std::endl;
}
