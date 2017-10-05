#include "planner.h"

using namespace std::chrono;
using json = nlohmann::json;


Planner::Planner() {
  ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}

double Planner::get_time_step() {
  milliseconds new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  diff = (double) (new_time-ms).count() / 1000;
  ms = new_time;

  return diff;
}

Planner::~Planner() {}

void Planner::update_traffic_state(json sensor_fusion) {
  double diff = get_time_step();

  predictions.clear();
  for (auto data : sensor_fusion) {
    Vehicle* vehicle = NULL;
    if (((double)data[5] <= Road::MAX_S) && ((double)data[6] >= 0)) {
      if (vehicles.find(data[0]) == vehicles.end()) {
        vehicle = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        vehicles[data[0]] = vehicle;
      }
      else {
        vehicle = vehicles[data[0]];
        (*vehicle).update_yaw(data[1], data[2], data[3], data[4], data[5], data[6], diff);
        if ((*vehicle).should_predict()) {
          std::vector<Prediction> car_preds = (*vehicle).generate_predictions(fsm.PRED_INTERVAL);
          predictions[(*vehicle).id] = car_preds;
        }
      }
    }
    else{
      auto it = vehicles.find(data[0]);
      if (it != vehicles.end()) {
        delete (*it).second;
        vehicles.erase((int)data[0]);
      }
    }
  }
}

void Planner::update_ego_state(double car_s, double x, double y, double yaw,
                               double s, double d, double speed) {
  original_yaw = yaw;
  fsm.car_s = car_s;
  fsm.ego.update_params(x, y, yaw, s, d, speed, diff);
}

void Planner::generate_trajectory(std::vector<double> previous_path_x,
                                  std::vector<double> previous_path_y) {
  fsm.update_state(predictions);
  fsm.realize_state(predictions);
  trajectory.set_previous_path(previous_path_x, previous_path_y);
  trajectory.generate_trajectory(fsm.car_s, ego.x_map, ego.y_map, original_yaw,
                                 ego.lane, fsm.get_expected_velocity());
}

std::vector<double> Planner::get_x_values() {
  return trajectory.next_x_vals;
}

std::vector<double> Planner::get_y_values() {
  return trajectory.next_y_vals;
}
