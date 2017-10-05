#include <vector>
#include "trajectory.h"
#include "road.h"
#include "spline.h"


Trajectory::Trajectory() {}

void Trajectory::set_previous_path(std::vector<double> previous_path_x,
                                   std::vector<double> previous_path_y) {
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
}

void Trajectory::convert2local(std::vector<double>& ptsx, std::vector<double>& ptsy) {
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
    ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
  }
}

Cartesian Trajectory::convert2global(double x, double y) {
  Cartesian coord;
  coord.x = (x*cos(ref_yaw) - y*sin(ref_yaw));
  coord.y = (x*sin(ref_yaw) + y*cos(ref_yaw));
  coord.x += ref_x;
  coord.y += ref_y;

  return coord;
}

void Trajectory::update_trajectory(std::vector<double> ptsx, std::vector<double> ptsy,
                                   double ref_vel) {
  convert2local(ptsx, ptsy);

  tk::spline s;
  s.set_points(ptsx, ptsy);

  double path_length = 50;
  for (int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = DISTANCE;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;
  double N = target_dist / (INTERVAL*mph2mps(ref_vel));
  for (int i = 1; i < path_length-previous_path_x.size(); i++) {
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    Cartesian point = convert2global(x_point, y_point);
    next_x_vals.push_back(point.x);
    next_y_vals.push_back(point.y);
  }
}

void Trajectory::generate_trajectory(double car_s, double original_x, double original_y,
                                     double original_yaw, int lane, double ref_vel) {
  next_x_vals.clear();
  next_y_vals.clear();

  std::vector<double> ptsx;
  std::vector<double> ptsy;

  if (abs(ref_vel) < 0.1) {
    return;
  }

  ref_x = original_x;
  ref_y = original_y;
  ref_yaw = Road::deg2rad(original_yaw);

  int prev_size = previous_path_x.size();
  if (prev_size < 2) {
    ref_yaw = Road::deg2rad(original_yaw);
    double prev_car_x = original_x - cos(original_yaw);
    double prev_car_y = original_y - sin(original_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(original_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(original_y);
  }
  else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  Cartesian next_wp0 = Road::getXY(car_s+DISTANCE, MIDDLE_LANE+LANE_WIDTH*lane);
  Cartesian next_wp1 = Road::getXY(car_s+2*DISTANCE, MIDDLE_LANE+LANE_WIDTH*lane);
  Cartesian next_wp2 = Road::getXY(car_s+3*DISTANCE, MIDDLE_LANE+LANE_WIDTH*lane);

  ptsx.push_back(next_wp0.x);
  ptsx.push_back(next_wp1.x);
  ptsx.push_back(next_wp2.x);
  ptsy.push_back(next_wp0.y);
  ptsy.push_back(next_wp1.y);
  ptsy.push_back(next_wp2.y);

  update_trajectory(ptsx, ptsy, ref_vel);
}
