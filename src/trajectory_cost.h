#ifndef TRAJECTORY_COST_H
#define TRAJECTORY_COST_H

#include <math.h>
#include <vector>
#include <map>
#include "vehicle.h"


enum CarState { CS = 0, KL = 1, PLCL = 2, PLCR = 3, LCL = 4, LCR = 5 };

struct Snapshot {
  double x_map;
  double y_map;
  double x_vel;
  double y_vel;
  double x_acc;
  double y_acc;
  double s_frenet;
  double d_frenet;
  double yaw;
  double ref_vel;
  int lane;
  int proposed_lane;
  CarState state;

  double get_speed() {
    return sqrt(x_vel*x_vel + y_vel*y_vel);
  }

  double get_acceleration() {
    return sqrt(x_acc*x_acc + y_acc*y_acc);
  }
};

class TrajectoryCost {
public:
  TrajectoryCost(double max_speed);

  virtual ~TrajectoryCost();

  double calculate_cost(double car_s, double ref_vel, std::vector<Snapshot> trajectory,
                        std::map<int, std::vector<Prediction> > predictions, CarState state);
private:
  struct Collider {
    bool collision;
    int time_step;
  };

  struct TrajectoryData {
    int proposed_lane;
    int current_lane;
    int end_lanes_from_goal;
    double avg_speed;
    double prop_closest_approach;
    double actual_closest_approach;
    Collider collides;
  };

  double MAX_SPEED;
  const int COLLISION = pow(10, 6);
  const int DANGER = 3*pow(10, 5);
  const int COMFORT = 0;//pow(10, 2);
  const int EFFICIENCY = 3*pow(10, 4);
  const double DESIRED_BUFFER = 20.0;
  const int PLANNING_HORIZON = 1;
  const double PRED_INTERVAL = 0.15;
  const double GOAL_LANE = 1;
  const double MANEUVER = 4.0;
  const double OBSERVED_DISTANCE = 65;
  const double MAX_DISTANCE = 999999;

  double change_lane_cost(std::vector<Snapshot> trajectory,
                          std::map<int, std::vector<Prediction> > predictions,
                          TrajectoryData data) const;

  double inefficiency_cost(std::vector<Snapshot> trajectory,
                           std::map<int, std::vector<Prediction> > predictions,
                           TrajectoryData data) const;

  double collision_cost(std::vector<Snapshot> trajectory,
                        std::map<int, std::vector<Prediction> > predictions,
                        TrajectoryData data) const;

  double buffer_cost(std::vector<Snapshot> trajectory,
                     std::map<int, std::vector<Prediction> > predictions,
                     TrajectoryData data) const;

  double free_line_cost(std::vector<Snapshot> trajectory,
                        std::map<int, std::vector<Prediction> > predictions,
                        TrajectoryData data) const;

  TrajectoryData get_helper_data(double car_s, double ref_s, std::vector<Snapshot> trajectory,
                                 std::map<int, std::vector<Prediction> > predictions,
                                 CarState state);

  bool will_collide_with(double car_s, double ref_speed, Snapshot snapshot,
                       Prediction s_now, CarState state, bool space);

  std::map<int, std::vector<Prediction> > filter_predictions_by_lane(
                                        std::map<int, std::vector<Prediction> > predictions,
                                        int lane);
};

#endif
