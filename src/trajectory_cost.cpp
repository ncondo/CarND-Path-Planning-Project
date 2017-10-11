#include <math.h>
#include <vector>
#include <algorithm>
#include "trajectory_cost.h"


TrajectoryCost::TrajectoryCost(double max_speed) {
  MAX_SPEED = max_speed;
}

TrajectoryCost::~TrajectoryCost() {}

double TrajectoryCost::change_lane_cost(std::vector<Snapshot> trajectory,
                                        std::map<int, std::vector<Prediction> > predictions,
                                        TrajectoryData data) const {
  if (data.proposed_lane != data.current_lane) {
    if (data.proposed_lane == 1) {
      return 0;
    }
    return COMFORT;
  }
  return 0;
}

double TrajectoryCost::inefficiency_cost(std::vector<Snapshot> trajectory,
                                         std::map<int, std::vector<Prediction> > predictions,
                                         TrajectoryData data) const {
  double speed = data.avg_speed;
  double target_speed = MAX_SPEED;
  double diff = target_speed - speed;
  double pct = diff / target_speed;
  double multiplier = pow(pct, 2);

  return 8*multiplier*EFFICIENCY;
}

double TrajectoryCost::collision_cost(std::vector<Snapshot> trajectory,
                                      std::map<int, std::vector<Prediction> > predictions,
                                      TrajectoryData data) const {
  if (data.collides.collision) {
    double time_til_collision = data.collides.time_step;
    double exponent = pow(float(time_til_collision), 2);
    double mult = exp(-exponent);

    return mult*COLLISION;
  }
  return 0;
}

double TrajectoryCost::buffer_cost(std::vector<Snapshot> trajectory,
                                   std::map<int, std::vector<Prediction> > predictions,
                                   TrajectoryData data) const {
  double closest = data.actual_closest_approach;
  if (closest < Vehicle::SAFE_DISTANCE/2) {
    return 3*DANGER;
  }

  if (closest > DESIRED_BUFFER) {
    return 0.0;
  }

  double multiplier = 1.0 - pow(closest/DESIRED_BUFFER, 2);

  return 3*multiplier*DANGER;
}

double TrajectoryCost::free_line_cost(std::vector<Snapshot> trajectory,
                                      std::map<int, std::vector<Prediction> > predictions,
                                      TrajectoryData data) const {
  double closest = data.prop_closest_approach;
  if (closest > OBSERVED_DISTANCE) {
    double multiplier = (MAX_DISTANCE-closest) / MAX_DISTANCE;
    return 20*multiplier*multiplier;
  }

  double multiplier = (OBSERVED_DISTANCE-closest) / OBSERVED_DISTANCE;
  return 5*multiplier*multiplier*COMFORT;
}

double TrajectoryCost::calculate_cost(double car_s, double ref_vel, std::vector<Snapshot> trajectory,
                                      std::map<int, std::vector<Prediction> > predictions,
                                      CarState state) {
  TrajectoryData data = get_helper_data(car_s, ref_vel, trajectory, predictions, state);
  double change_cost = change_lane_cost(trajectory, predictions, data);
  double ineff_cost = inefficiency_cost(trajectory, predictions, data);
  double coll_cost = collision_cost(trajectory, predictions, data);
  double buff_cost = buffer_cost(trajectory, predictions, data);
  double free_cost = free_line_cost(trajectory, predictions, data);
  double total_cost = change_cost + ineff_cost + coll_cost + buff_cost + free_cost;

  return total_cost;
}

TrajectoryCost::TrajectoryData TrajectoryCost::get_helper_data(double car_s, double ref_s,
                                              std::vector<Snapshot> trajectory,
                                              std::map<int, std::vector<Prediction> > predictions,
                                              CarState state) {
  TrajectoryData data = TrajectoryData();
  Snapshot current_snapshot = trajectory[0];
  Snapshot first = trajectory[1];
  Snapshot last = trajectory[trajectory.size()-1];

  double dt = trajectory.size()*PRED_INTERVAL;
  data.current_lane = first.lane;
  data.proposed_lane = last.proposed_lane;
  data.avg_speed = (last.get_speed()*dt - current_snapshot.get_speed()) / dt;
  data.prop_closest_approach = MAX_DISTANCE;
  data.actual_closest_approach = MAX_DISTANCE;
  data.collides = Collider();
  data.collides.collision = false;
  bool check_collisions = current_snapshot.lane != data.proposed_lane;

  std::map<int, std::vector<Prediction> > cars_in_proposed_lane = filter_predictions_by_lane(predictions,
                                                                                             data.proposed_lane);
  std::map<int, std::vector<Prediction> > cars_in_actual_lane = filter_predictions_by_lane(predictions,
                                                                                           data.current_lane);
  for (int i = 0; i < PLANNING_HORIZON; i++) {
    Snapshot snapshot = trajectory[i];
    for (auto pair : cars_in_actual_lane) {
      Prediction pred = pair.second[i];
      double dist = -pred.get_distance(snapshot.x_map, snapshot.y_map, snapshot.s_frenet);
      if (dist >= 0 && dist < data.actual_closest_approach) {
        data.actual_closest_approach = dist;
      }
    }
  }

  for (int i = 0; i < PLANNING_HORIZON; i++) {
    Snapshot snapshot = trajectory[i];
    for (auto pair : cars_in_proposed_lane) {
      Prediction pred = pair.second[i];
      double dist = -pred.get_distance(snapshot.x_map, snapshot.y_map, snapshot.s_frenet);
      if (check_collisions) {
        bool vehicle_collides = will_collide_with(car_s, ref_s, snapshot, pred, state,
                                                data.actual_closest_approach < MANEUVER);
        if (vehicle_collides) {
          data.collides.collision = true;
          data.collides.time_step = i;
        }
        else if (car_s > pred.s_frenet) {
          dist = MAX_DISTANCE;
        }
      }
      if (dist >= 0 && dist < data.prop_closest_approach) {
        data.prop_closest_approach = dist;
        if (data.proposed_lane == data.current_lane) {
          data.actual_closest_approach = data.prop_closest_approach;
        }
      }
    }
  }
  return data;
}

bool TrajectoryCost::will_collide_with(double car_s, double ref_speed, Snapshot snapshot,
                                     Prediction s_now, CarState state, bool space) {
  double s_frenet = snapshot.s_frenet;
  double vel = snapshot.get_speed();
  double collide_car_vel = s_now.get_velocity();
  double diff = s_now.get_distance(snapshot.x_map, snapshot.y_map, snapshot.s_frenet);
  double prediction_time = 4 / snapshot.x_vel;
  if (car_s > s_now.s_frenet) {
    double predicted_distance1v = diff + prediction_time*(vel-collide_car_vel);
    double predicted_distance2v = diff + 10*PRED_INTERVAL*(ref_speed-collide_car_vel);
    if ((predicted_distance2v < MANEUVER || predicted_distance1v < MANEUVER || space || diff < -1.0)) {
      return true;
    }
  }
  else {
    double predicted_distance1v = -diff + 3*PRED_INTERVAL*(collide_car_vel - vel);
    if (predicted_distance1v < 0 || -diff < -MANEUVER) {
      return true;
    }
  }
  return false;
}

std::map<int, std::vector<Prediction> > TrajectoryCost::filter_predictions_by_lane(
                                              std::map<int, std::vector<Prediction> > predictions,
                                              int lane) {
  std::map<int, std::vector<Prediction> > filtered = {};
  for (auto pair : predictions) {
    if (pair.second[0].lane == lane) {
      filtered[pair.first] = pair.second;
    }
    return filtered;
  }
}
