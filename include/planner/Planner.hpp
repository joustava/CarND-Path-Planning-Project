#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <vector>
#include <nlohmann/json.hpp>
#include "spline.h"

class Planner {
  private:
  /**
   * @brief map data
   * 
   */
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  
  /**
   * @brief car data
   * 
   */
  double car_x_;
  double car_y_;
  double car_s_;
  double car_d_;
  double car_yaw_;
  double car_speed_;

  /**
   * @brief previous path data
   * 
   */
  nlohmann::json previous_path_x_;
  double prev_size;
  nlohmann::json previous_path_y_;
  
  /**
   * @brief
   * 
   */
  static constexpr double const& SAFE_DIST_FRONT = 30.0;
  static constexpr double const& SAFE_DIST_REAR = 20.0;

  /**
   * @brief The Velocity we aim to drive at
   * 
   */
  static constexpr double const& REF_VEL = 49.5;

  /**
   * @brief Velocity delta to in- or decrease the velocity in steps. 
   * 
   */
  static constexpr double const& DELTA_VEL = 0.224;
  
  /**
   * @brief Lane index 0..2 left to right.
   * 
   */
  int current_lane = 1;
  
  double velocity = 0.0;
  tk::spline spline;

   /**
   * @brief Initializes the planner with Map data from file.
   * 
   * @param filename defaults to "../data/highway_map.csv"
   */
  void load_map(std::string filename);

  /**
   * @brief Check for cars in direction
   * 
   * @param s frenet param of detected car
   * @param d frenet param of detected car
   * @param direction side to check
   * @return true car detected at side
   * @return false no car detected at side
   */
  bool proximity(double s, double d, int side);

  public:

  Planner(std::string filename);
  ~Planner() = default;

  /**
   * @brief 
   * 
   * @param car_x 
   * @param car_y 
   * @param car_s 
   * @param car_d 
   * @param car_yaw 
   * @param car_speed 
   * @param previous_path_x 
   * @param previous_path_y 
   * @param end_path_s 
   */
  void update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, nlohmann::json previous_path_x, nlohmann::json previous_path_y, double end_path_s);
  
  /**
   * @brief 
   * 
   * @param sensor_fusion 
   */
  void track(nlohmann::json sensor_fusion);
  
  /**
   * @brief 
   * 
   * @param next_x_vals 
   * @param next_y_vals 
   */
  void plan(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals); 
};

#endif