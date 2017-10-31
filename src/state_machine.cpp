#include <algorithm>
#include <string>

#include "state_machine.h"
#include "vehicle.h"


StateMachine::StateMachine(Vehicle& ego) : ego(ego) {
  proposed_lane = ego.lane;
};

StateMachine::~StateMachine() {};

double StateMachine::get_expected_velocity() {
  return ref_vel;
}

void StateMachine::update_state(std::map<int, std::vector<Prediction> > predictions) {
  state = get_next_state(predictions);
}

CarState StateMachine::get_next_state(std::map<int, std::vector<Prediction> > predictions) {
  std::vector<CarState> states;
  if (state == CarState::PLCL) {
    states = std::vector<CarState> { CarState::KL, CarState::PLCL, CarState::LCL };
  }
  else if (state == CarState::PLCR) {
    states = std::vector<CarState> { CarState::KL, CarState::PLCR, CarState::LCR };
  }
  else if (state == CarState::LCL) {
    states = std::vector<CarState> { CarState::KL };
  }
  else if (state == CarState::LCR) {
    states = std::vector<CarState> { CarState::KL };
  }
  else {
    states = std::vector<CarState> { CarState::KL };
    if (ego.lane > 0) {
      states.push_back(CarState::PLCL);
    }
    if (ego.lane < Road::NUM_LANES-1) {
      states.push_back(CarState::PLCR);
    }
  }

  if (states.size() == 1) {
    return states[0];
  }

  double best_cost = 999999.0;
  CarState best_state;
  for (int i = 0; i < states.size(); i++) {
    std::map<int, std::vector<Prediction> > predictions_copy = predictions;
    std::vector<Snapshot> trajectory = trajectory_for_state(states[i], predictions_copy,
                                                            PRED_COUNT);
    double cost = trajectory_cost.calculate_cost(car_s, ref_vel, trajectory,
                                                 predictions, states[i]);
    if (cost < best_cost) {
      best_cost = cost;
      best_state = states[i];
    }
  }
  return best_state;
}

std::vector<Snapshot> StateMachine::trajectory_for_state(CarState proposed_state,
                                                         std::map<int, std::vector<Prediction> > predictions,
                                                         int horizon) {
  Snapshot snapshot = get_snapshot();
  std::vector<Snapshot> trajectory {snapshot};
  for (int i = 0; i < horizon; i++) {
    restore_state_from_snapshot(snapshot);
    state = proposed_state;
    realize_state(predictions);
    ego.increment(i*PRED_INTERVAL);
    trajectory.push_back(get_snapshot());

    for (auto pair : predictions) {
      auto pred = pair.second;
      pred.erase(pred.begin());
    }
  }
  restore_state_from_snapshot(snapshot);

  return trajectory;
}

void StateMachine::realize_state(std::map<int, std::vector<Prediction> > predictions) {
  if (state == CarState::CS) {
    realize_constant_speed();
  }
  else if (state == CarState::KL) {
    realize_keep_lane(predictions);
  }
  else if (state == CarState::LCL) {
    realize_lane_change(predictions, "L");
  }
  else if (state == CarState::LCR) {
    realize_lane_change(predictions, "R");
  }
  else if (state == CarState::PLCL) {
    realize_prep_lane_change(predictions, "L");
  }
  else if (state == CarState::PLCR) {
    realize_prep_lane_change(predictions, "R");
  }
}

void StateMachine::realize_constant_speed() {}

void StateMachine::update_ref_vel(std::map<int, std::vector<Prediction> > predictions, int lane) {
  bool too_close = false, keep_speed = false, danger = false;
  double max_speed = MAX_SPEED;
  for (auto pair : predictions) {
    Prediction pred = pair.second[0];
    double target_speed = pred.get_velocity();
    if (ego.is_behind(pred, lane) && target_speed < max_speed) {
      max_speed = target_speed - SPEED_INCREMENT;
      keep_speed = true;
    }
    if (ego.is_close(pred, lane)) {
      too_close = true;
      if (pred.s_frenet < ego.s_frenet + 5) {
        danger = true;
      }
    }
  }

  double velocity = ref_vel;
  if (too_close) {
    if (danger) {
      if (velocity > 40.0) {
        velocity -= 2*SPEED_INCREMENT;
      }
      else {
        velocity -= SPEED_INCREMENT;
      }
    }
    else {
      if (velocity < max_speed) {
        velocity += SPEED_INCREMENT;
      }
      else if (velocity > max_speed) {
        if (velocity > 40.0) {
          velocity -= 2*SPEED_INCREMENT;
        }
        else {
          velocity -= SPEED_INCREMENT;
        }
      }
    }
  }
  else {
    if (keep_speed && velocity > 25 && velocity > max_speed*2.7) {
      velocity -= SPEED_INCREMENT;
    }
    else {
      velocity += SPEED_INCREMENT;
    }
    if (velocity > MAX_SPEED) {
      velocity = MAX_SPEED;
    }
  }
  if (velocity < 0.0) {
    velocity = 0.0;
  }
  ref_vel = velocity;
}

void StateMachine::realize_keep_lane(std::map<int, std::vector<Prediction> > predictions) {
  proposed_lane = ego.lane;
  update_ref_vel(predictions, proposed_lane);
}

void StateMachine::realize_lane_change(std::map<int, std::vector<Prediction> > predictions,
                                       std::string direction) {
  int delta = -1;
  if (direction.compare("R") == 0) {
    delta = 1;
  }
  ego.lane += delta;
  proposed_lane = ego.lane;
  update_ref_vel(predictions, proposed_lane);
}

void StateMachine::realize_prep_lane_change(std::map<int, std::vector<Prediction> > predictions,
                                            std::string direction) {
  int delta = -1;
  bool close = false;
  if (direction.compare("R") == 0) {
    delta = 1;
  }
  proposed_lane = ego.lane + delta;

  std::vector<std::vector<Prediction> > at_behind;
  for (auto pair : predictions) {
    int v_id = pair.first;
    std::vector<Prediction> v = pair.second;
    if (ego.is_ahead(v[0], proposed_lane)) {
      at_behind.push_back(v);
    }
    if (ego.is_close(v[0], ego.lane)) {
      if (v[0].get_distance(ego.x_map, ego.y_map, ego.s_frenet) < 4) {
        close = true;
      }
    }
  }
  if (at_behind.size() > 0) {
    double velocity = ref_vel;
    if (close) {
      if (velocity > 40.0) {
        velocity -= 2*SPEED_INCREMENT;
      }
      else {
        velocity -= SPEED_INCREMENT;
      }
    }
    else {
      velocity += SPEED_INCREMENT;
    }
    if (velocity > MAX_SPEED) {
      velocity = MAX_SPEED;
    }
    ref_vel = velocity;
  }
}

Snapshot StateMachine::get_snapshot() {
  Snapshot snapshot;
  snapshot.x_map = this->ego.x_map;
  snapshot.y_map = this->ego.y_map;
  snapshot.x_vel = this->ego.x_vel;
  snapshot.y_vel = this->ego.y_vel;
  snapshot.s_frenet = this->ego.s_frenet;
  snapshot.d_frenet = this->ego.d_frenet;
  snapshot.x_acc = this->ego.x_acc;
  snapshot.y_acc = this->ego.y_acc;
  snapshot.yaw = this->ego.yaw;
  snapshot.lane = this->ego.lane;
  snapshot.state = this->state;
  snapshot.ref_vel = this->ref_vel;
  snapshot.proposed_lane = this->proposed_lane;

  return snapshot;
}

void StateMachine::restore_state_from_snapshot(Snapshot snapshot) {
  this->ego.x_map = snapshot.x_map;
  this->ego.y_map = snapshot.y_map;
  this->ego.x_vel = snapshot.x_vel;
  this->ego.y_vel = snapshot.y_vel;
  this->ego.s_frenet = snapshot.s_frenet;
  this->ego.d_frenet = snapshot.d_frenet;
  this->ego.x_acc = snapshot.x_acc;
  this->ego.y_acc = snapshot.y_acc;
  this->ego.yaw = snapshot.yaw;
  this->ego.lane = snapshot.lane;
  this->state = snapshot.state;
  this->ref_vel = snapshot.ref_vel;
  this->proposed_lane = snapshot.proposed_lane;
}
