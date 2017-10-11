#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <vector>
#include <string>
#include <map>
#include "trajectory_cost.h"
#include "vehicle.h"


class StateMachine {
public:
  StateMachine(Vehicle& ego);

  ~StateMachine();

  Vehicle& ego;
  double car_s;
  const double PRED_INTERVAL = 0.15;

  double get_expected_velocity();

  void update_state(std::map<int, std::vector<Prediction> > predictions);

  void realize_state(std::map<int, std::vector<Prediction> > predictions);

private:
  const double SPEED_INCREMENT = 0.224;
  const double MAX_SPEED = 49.5;
  const double TIME_INTERVAL = 0.02;
  const double PRED_COUNT = 5;
  double ref_vel = 0;
  int proposed_lane;
  CarState state = CarState::CS;

  TrajectoryCost trajectory_cost = TrajectoryCost(MAX_SPEED);

  void realize_constant_speed();

  void realize_keep_lane(std::map<int, std::vector<Prediction> > predictions);

  void realize_lane_change(std::map<int, std::vector<Prediction> > predictions,
                           std::string direction);

  void realize_prep_lane_change(std::map<int, std::vector<Prediction> > predictions,
                                std::string direction);

  void update_ref_vel(std::map<int, std::vector<Prediction> > predictions, int lane);

  CarState get_next_state(std::map<int, std::vector<Prediction> > predictions);

  std::vector<Snapshot> trajectory_for_state(CarState state,
                                             std::map<int, std::vector<Prediction> > predictions,
                                             int horizon);

  void restore_state_from_snapshot(Snapshot snapshot);

  Snapshot get_snapshot();
};

#endif
