#ifndef PLANNER_H
#define PLANNER_H

#include <math.h>
#include <vector>
#include <map>
#include <chrono>
#include "json.hpp"
#include "vehicle.h"
#include "state_machine.h"
#include "trajectory.h"

using namespace std::chrono;
using json = nlohmann::json;


class Planner {
public:
  Planner();

  virtual ~Planner();

  void update_traffic_state(json sensor_fusion);

  void update_ego_state(double car_s, double x, double y, double yaw, double s,
                        double d, double speed);

  void generate_trajectory(std::vector<double> previous_path_x,
                           std::vector<double> previous_path_y);

  std::vector<double> get_x_values();

  std::vector<double> get_y_values();

private:
  double original_yaw;
  double diff;
  milliseconds ms;
  std::map<int, Vehicle*> vehicles;
  std::map<int, std::vector<Prediction> > predictions;
  Vehicle ego = Vehicle(-1);
  StateMachine fsm = StateMachine(ego);
  Trajectory trajectory = Trajectory();
  double get_time_step();
};

#endif
