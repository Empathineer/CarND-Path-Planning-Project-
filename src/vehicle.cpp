/*
 * vehicle.cpp
 *
 *  Created on: 26 Jul 2020
 *      Author: Carissa 
 */

#include "vehicle.h"
#include "helpers.h"
#include "constants.h"


/*
 * Vehicle
 */
Vehicle::Vehicle(){}
Vehicle::Vehicle(const int id) {
  this->id = id;
}

Vehicle::~Vehicle(){}


/*
 * EgoVehicle
 */
EgoVehicle::EgoVehicle() : Vehicle::Vehicle(0){}

void EgoVehicle::set_state_from_simulator_json(const nlohmann::json &j) {
  this->state.x = j[1]["x"];
  this->state.y = j[1]["y"];
  this->state.yaw = j[1]["yaw"];
  this->state.s = j[1]["s"];
  this->state.d = j[1]["d"];
  this->state.v = mph2ms(j[1]["speed"]);

  // determine the sdc's current lane using known lane width 
  this->state.lane = (int) this->state.d / LANE_WIDTH;
}

// Set remainder of previous path to the sdc's previous path but also store 
// the last Frenet pts as a reference. 
void EgoVehicle::set_previous_path_from_simulator_json(const nlohmann::json &j) {
  this->prev_path = Path::previous_path_from_json(j);
}

// Will return vector of other detected vehicle objects,
// containing their states (x, y, vx, vy, s, d).
void EgoVehicle::detect_other_vehicles_from_sensor_json(const nlohmann::json &j) {
  this->other_vehicles = OtherVehicle::from_sensor_fusion_json(j);
}


bool EgoVehicle::get_vehicle_in_lane(int search_lane, bool search_ahead, OtherVehicle &vehicle){
  
  bool found_vehicle = false;
  double dist_min = numeric_limits<double>::infinity(); 

  int prev_path_size = this->prev_path.size();
  double s_ref;
  double t_ref = prev_path_size * (double)PATH_DT;
  
  if (prev_path_size > 0) {
    s_ref = this->prev_path.end_s;
  }
  else {
    s_ref = this->state.s;
  }

  for (int i = 0; i < this->other_vehicles.size(); i++) {
    OtherVehicle other_vehicle = this->other_vehicles[i];
    //hold state of 
    State other_vehicle_pred_state = other_vehicle.predict_state(t_ref); 
    bool in_lane = other_vehicle_pred_state.lane == search_lane;
    
    // Determine if other car in lane is ahead or behind 
    bool flag;
    if (search_ahead) {
      flag = other_vehicle_pred_state.s > s_ref;
    } else {
      flag = other_vehicle_pred_state.s < s_ref;
    }
    
    //calculate distance between sdc and car ahead or behind
    double dist = fabs(other_vehicle_pred_state.s - s_ref); 
    
    // There is a vehicle in the sdc's lane if :
    // 1) other car is in the same lane 
    // 2) other car is ahead 
    // 3) other car's distance ahead is less than the min following distance
    if (in_lane && flag && (dist < dist_min)) {
      dist_min = dist;        
      vehicle = other_vehicle;      
      found_vehicle = true;
    }
  }

  string debug_case;
  if (search_ahead) {
    debug_case = "ahead";
  } else {
    debug_case = "behind";
  }
  
  if (found_vehicle) {
    cout << "lane " << search_lane << " vehicle " << debug_case << ": " << dist_min << endl;
  } else {
    cout << "lane " << search_lane << " vehicle " << debug_case << ": NA" << endl;
  }

  return found_vehicle;
}

LaneKinematics EgoVehicle::get_lane_kinematics(int search_lane) {

  double gap_ahead = numeric_limits<double>::infinity();
  double gap_behind = numeric_limits<double>::infinity();
  double lane_velocity = EGO_MAX_VEL;

  int prev_path_size = this->prev_path.size();
  double s_ref;
  double t_ref = prev_path_size * (double)PATH_DT;
  
  if (prev_path_size > 0) {
    s_ref = this->prev_path.end_s;
  }
  else {
    s_ref = this->state.s;
  }

  // get vehicle ahead in query lane
  OtherVehicle vehicle_ahead;
  bool found_vehicle_ahead = this->get_vehicle_in_lane(
    search_lane, 
    true, 
    vehicle_ahead
  );

  // get vehicle behind in query lane
  OtherVehicle vehicle_behind;
  bool found_vehicle_behind = this->get_vehicle_in_lane(
    search_lane, 
    false, 
    vehicle_behind
  );

  if (found_vehicle_ahead) {
    State ahead_vehicle_predicted_state = vehicle_ahead.predict_state(t_ref);
    gap_ahead = ahead_vehicle_predicted_state.s - s_ref;
    lane_velocity = ahead_vehicle_predicted_state.v;
  } 
  
  if (found_vehicle_behind) {
    State behind_vehicle_predicted_state = vehicle_behind.predict_state(t_ref);
    gap_behind = s_ref - behind_vehicle_predicted_state.s;
    
    if (!found_vehicle_ahead) {
      lane_velocity = behind_vehicle_predicted_state.v;
    }
  }

  LaneKinematics lk_found_vehicle;
  lk_found_vehicle.id = search_lane;
  lk_found_vehicle.gap_ahead = gap_ahead;
  lk_found_vehicle.gap_behind = gap_behind;
  lk_found_vehicle.v = lane_velocity;
  return lk_found_vehicle;
}


/*
 * OtherVehicle
 */
OtherVehicle::OtherVehicle() : Vehicle::Vehicle(){}
OtherVehicle::OtherVehicle(const int id) : Vehicle::Vehicle(id){}

//Predicts state of neighboring vehicle, assuming
// 1) constant velocity 
// 2) no lane change 
State OtherVehicle::predict_state(const double t_horizon) {
  State current_state = this->state;
  State pred_state;
  pred_state.s = current_state.s + current_state.v * t_horizon;
  pred_state.d = current_state.d;
  pred_state.v = current_state.v;
  pred_state.lane = current_state.lane;

  return pred_state;
}

//retrieve the state of neighboring car 
vector<OtherVehicle> OtherVehicle::from_sensor_fusion_json(const nlohmann::json &j) {
  auto sensor_fusion = j[1]["sensor_fusion"];
  vector<OtherVehicle> other_vehicles;
  
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // instantiate vehicle via id
    int id = sensor_fusion[i][0];
    OtherVehicle other_vehicle = OtherVehicle(id);

    // retrieve position and velocity from sensor measurement
    State state;
    state.x = sensor_fusion[i][1];
    state.y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    state.s = sensor_fusion[i][5];
    state.d = sensor_fusion[i][6];
    
    state.v = sqrt(vx * vx + vy * vy);

    // Calculate lane number based on Frenet normal d coord and lane width
    state.lane = (int) state.d / LANE_WIDTH;

    // set neighboring vehicle state
    other_vehicle.state = state;

    other_vehicles.push_back(other_vehicle);
  }

  return other_vehicles;
}

