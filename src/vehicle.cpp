#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "vehicle.h"


Vehicle::Vehicle(int id, double x_map, double y_map, double x_vel,
                 double y_vel, double s_frenet, double d_frenet) {

  this->id = id;
  this->x_map = x_map;
  this->y_map = y_map;
  this->x_vel = x_vel;
  this->y_vel = y_vel;
  this->s_frenet = s_frenet;
  this->d_frenet = d_frenet;
  this->yaw = get_yaw(x_vel, y_vel);
  this->x_acc = 0.0;
  this->y_acc = 0.0;
  this->lane = get_lane(d_frenet);
}

Vehicle::~Vehicle() {}

double Vehicle::get_velocity() {
  return sqrt(this->x_vel*this->x_vel + this->y_vel*this->y_vel);
}

double Vehicle::get_yaw(double x_vel, double y_vel) {
  double angle = atan2(y_vel, x_vel);
  return (abs(angle) < 0.1) ? 0.0 : angle;
}

int Vehicle::get_lane(double d_frenet) {
  return (int) d_frenet / Road::LANE_WIDTH;
}

void Vehicle::increment(double dt) {
  if (abs(this->x_acc) < 0.001) {
    this->x_map += this->x_vel * dt;
  }
  else {
    this->x_map += this->x_vel * dt + (this->x_acc * dt * dt) / 2;
    this->x_vel += this->x_acc * dt;
  }

  if (abs(this->y_acc) < 0.001) {
    this->y_map += this->y_vel * dt;
  }
  else {
    this->y_map += this->y_vel * dt + (this->y_acc * dt * dt) / 2;
    this->y_vel += this->y_acc * dt;
  }

  this->yaw = get_yaw(this->x_vel, this->y_vel);
  Frenet frenet = Road::getFrenet(this->x_map, this->y_map, this->yaw);
  this->s_frenet = frenet.s;
  this->d_frenet = frenet.d;
}

Prediction Vehicle::state_at(double dt) {
  Prediction pred;
  if (abs(this->x_acc) < 0.001) {
    pred.x_map = this->x_map + this->x_vel * dt;
    pred.x_vel = this->x_vel;
  }
  else {
    pred.x_map = this->x_map + this->x_vel * dt + (this->x_acc * dt * dt) / 2;
    pred.x_vel = this->x_vel + this->x_acc * dt;
  }

  if (abs(this->y_acc) < 0.001) {
    pred.y_map = this->y_map + this->y_vel * dt;
    pred.y_vel = this->y_vel;
  }
  else {
    pred.y_map = this->y_map + this->y_vel * dt + (this->y_acc * dt * dt) / 2;
    pred.y_vel = this->y_vel + this->y_acc * dt;
  }

  double yaw = get_yaw(pred.x_vel, pred.y_vel);
  Frenet frenet = Road::getFrenet(pred.x_map, pred.y_map, yaw);
  pred.s_frenet = frenet.s;
  pred.d_frenet = frenet.d;
  pred.lane = get_lane(pred.d_frenet);

  return pred;
}

std::vector<Prediction> Vehicle::generate_predictions(double interval, int horizon) {
  std::vector<Prediction> predictions;
  for (int i = 0; i < horizon; ++i) {
    predictions.push_back(state_at(i*interval));
  }
  return predictions;
}
