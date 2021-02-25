#include "Planner.hpp"
#include "helpers.h"
#include <math.h>

std::vector<double> planX(double car_x, double car_yaw) {
    double dist_inc = 0.5;
    std::vector<double> next_x_vals;
    for (int i = 0; i < 50; ++i) {
      next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
      // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
    return next_x_vals;
}

std::vector<double> planY(double car_y, double car_yaw) {
    double dist_inc = 0.5;
    std::vector<double> next_y_vals;
    for (int i = 0; i < 50; ++i) {
      // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
      next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
    return next_y_vals;
}