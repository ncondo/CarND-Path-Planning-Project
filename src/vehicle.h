#ifndef VEHICLE_H
#define VEHICLE_H

#include <random>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

#include "road.h"


struct Prediction {
  double x_map;
  double y_map;
  double x_vel;
  double y_vel;
  double d_frenet;
  double s_frenet;
  int lane;

  double get_velocity() {
    return sqrt(x_vel*x_vel + y_vel*y_vel);
  }

  double get_distance(double other_x, double other_y, double other_s) {
    double diff_car = sqrt((x_map-other_x)*(x_map-other_x) + (y_map-other_y)*(y_map-other_y));
    double diff_frenet = other_s - s_frenet;
    if (diff_car - abs(diff_frenet) < 100) {
      return diff_frenet;
    }
    else {
      return copysign(diff_car, diff_frenet);
    }
  }

};

class Vehicle {
public:
  int id;
  double x_map;
  double y_map;
  double x_vel;
  double y_vel;
  double s_frenet;
  double d_frenet;
  double yaw;
  double x_acc;
  double y_acc;
  int lane = 1;

  Vehicle(int id);

  Vehicle(int id, double x_map, double y_map, double x_vel,
          double y_vel, double s_frenet, double d_frenet);

  virtual ~Vehicle();

  double get_velocity();

  bool should_predict();

  double get_yaw(double x_vel, double y_vel);

  int get_lane(double d_frenet);

  void update_params(double x_map, double y_map, double yaw, double s_frenet,
                     double d_frenet, double speed, double diff);

  void update_acc(double vel_x, double vel_y, double diff);

  void update_yaw(double x_map, double y_map, double x_vel, double y_vel,
                  double s_frenet, double d_frenet, double diff);

  void increment(double dt);

  bool is_ahead(Prediction pred, int checked_lane);

  bool is_behind(Prediction pred, int checked_lane);

  bool is_close(Prediction pred, int checked_lane);

  Prediction state_at(double dt);

  std::vector<Prediction> generate_predictions(double interval, int horizon = 10);
};

#endif
