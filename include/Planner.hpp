#ifndef PLANNER_H_
#define PLANNER_H_

#include <vector>


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
  double previous_path_x;
  double previous_path_y;
  // Previous path's end s and d values 
  double end_path_s;
  double end_path_d;

  public:
  Planner();
  ~Planner() = default;

  /**
   * @brief Initializes the planner with Map data from file.
   * 
   * @param filename defaults to "../data/highway_map.csv"
   */
  void load_map(std::string filename);
  void updateCar(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
  // void updatePath(double previous_path_x, double previous_path_y, double end_path_s, double end_path_d);
  std::vector<double> planX();
  std::vector<double> planY();
};

#endif