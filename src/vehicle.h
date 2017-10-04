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
  int lane;

  Vehicle(int id, double x_map, double y_map, double x_vel,
          double y_vel, double s_frenet, double d_frenet);

  virtual ~Vehicle();

  double get_velocity();

  double get_yaw(double x_vel, double y_vel);

  int get_lane(double d_frenet);

  void increment(double dt);

  Prediction state_at(double dt);

  std::vector<Prediction> generate_predictions(double interval, int horizon);
};

#endif
