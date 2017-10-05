#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "road.h"
#include "vehicle.h"


class Trajectory {
public:
  Trajectory();

  virtual ~Trajectory() {}

  double ref_x;
  double ref_y;
  double ref_yaw;
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;

  void set_previous_path(std::vector<double> previous_path_x,
                         std::vector<double> previous_path_y);
  void generate_trajectory(double car_s, double car_x, double car_y,
                           double car_yaw, int lane, double ref_vel);

private:
  const double INTERVAL = 0.02;
  const double DISTANCE = 30;
  const double LANE_WIDTH = 4;
  const double MIDDLE_LANE = LANE_WIDTH/2;
  const double MIN_SPEED = 0.3;

  double mph2mps(double mph) { return mph / 2.24; }

  void convert2local(std::vector<double>& ptsx, std::vector<double>& ptsy);

  Cartesian convert2global(double x, double y);

  void update_trajectory(std::vector<double> ptsx, std::vector<double> ptsy,
                         double ref_vel);
};

#endif
