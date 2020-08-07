/*
 * vehicle.h
 *
 *  Created on: 26 Jul 2020
 *      Author: Carissa Chan
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <string>
#include <vector>
#include <limits>
// #include "base.h"
#include "helpers.h"

using namespace std;

struct Vehicle{
  int id;
  State state;
  
  Vehicle();
  Vehicle(const int id);
  virtual ~Vehicle();

};

struct OtherVehicle : Vehicle{
  OtherVehicle();
  OtherVehicle(const int id);
  State predict_state(const double t_horizon);
  static vector<OtherVehicle> from_sensor_fusion_json(const nlohmann::json &j);
};

struct EgoVehicle : Vehicle{
  Path prev_path;
  FSMState fsm_state;
  vector<OtherVehicle> other_vehicles;

  EgoVehicle();
  void set_state_from_simulator_json(const nlohmann::json &j);
  void set_previous_path_from_simulator_json(const nlohmann::json &j);
  void detect_other_vehicles_from_sensor_json(const nlohmann::json &j);
  bool get_vehicle_in_lane(int search_lane, bool search_ahead, OtherVehicle &vehicle);
  LaneKinematics get_lane_kinematics(int search_lane);
};



#endif /* VEHICLE_H_ */