/*
 * lane_tracker.cpp
 *
 *  Created on: 26 Jul 2020
 *      Author: Carissa Chan
 */

#include "lane_tracker.h"
#include "constants.h"
#include "helpers.h"

LaneTracker::LaneTracker(){}

int LaneTracker::getLane(const double d, const double laneWidth) {
    for (int lane = 0; lane <= 2; lane++) {
        if (d > laneWidth * lane && d <= laneWidth * (lane + 1)) {
            return lane;
        }
    }
    cout << "couldn't find lane for d = " << d << endl;
    return 0;
}

double LaneTracker::getLaneOffsetD(const int lane_number, const double laneWidth) {
    return (laneWidth * lane_number) + 2.0;
}

bool LaneTracker::is_lane_safe(LaneKinematics lane_kinematics) {
  double safety_gap_ahead = 25;
  double safety_gap_behind = 10;

  bool gap_ahead_safe = lane_kinematics.gap_ahead > safety_gap_ahead;
  bool gap_behind_safe = lane_kinematics.gap_behind > safety_gap_behind;

  bool safe_lane = gap_ahead_safe && gap_behind_safe;
  return safe_lane;
}

FSMState LaneTracker::get_target_state(EgoVehicle &car) {
  double dist_pass = 20; //space in betweem ego and front OtherVehicle to 
  double deltav_safe_factor = 0.85; //safety factor 
  double deltav_change_lane = 0.8 // reduce speed factor when changing lanes
  double d_tol = 0.5; 
  FSMState car_fs;

  //Decide whether to change lanes based on whether there is enough room to 
  if (car.fsm_state.changing_lane) {
    double d_target = 2 + 4 * car.fsm_state.lane_target; //calculate normal Frenet coordinate per lane change 
    double d_diff = fabs(car.prev_path.end_d - d_target); 
    if (d_diff > d_tol) {
      return car.fsm_state;
    }
    car.fsm_state.changing_lane = false;
  }

  // Tracking neighboring cars in the left, right, and current lane. 
  LaneKinematics left_lane;
  LaneKinematics right_lane;
  LaneKinematics current_lane = car.get_lane_kinematics(car.state.lane);
  
  if (current_lane.gap_ahead < dist_pass) {
    // See if it's possible to change lanes 
    switch(current_lane.id) {
      case 0:
        // Check if it's safe to change into right lane. 
        right_lane = car.get_lane_kinematics(current_lane.id + 1); 
        
        if (this->is_lane_safe(right_lane)) {
          // TURN RIGHT
          car_fs.lane_target = right_lane.id;
          car_fs.v_target = right_lane.v * deltav_change_lane;
          car_fs.changing_lane = true;
        } else {
          // STAY, with a safety factor (lower velocity)
          car_fs.lane_target = current_lane.id;
          car_fs.v_target = current_lane.v * deltav_safe_factor;
          car_fs.changing_lane = false;
        }
        break;
        
      // Determine which way to turn based on which lane is "open" 
      // If the car in that lane provides sufficient distance ahead 
      // AND behind, it is safe to change into. 
      case 1:
        // Check if it's safe to switch to left lane 
        left_lane = car.get_lane_kinematics(current_lane.id - 1); 
        if (this->is_lane_safe(left_lane)) {
          // TURN LEFT
          car_fs.lane_target = left_lane.id;
          car_fs.v_target = left_lane.v * deltav_change_lane;
          car_fs.changing_lane = true;
        } else {
          // Check if it's safe to switch to right lane 
          right_lane = car.get_lane_kinematics(current_lane.id + 1);
          if (this->is_lane_safe(right_lane)) {
            // TURN RIGHT
            car_fs.lane_target = right_lane.id;
            car_fs.v_target = right_lane.v * deltav_change_lane;
            car_fs.changing_lane = true;
          } else {
            // STAY but maintain a lower velocity by a safety factor
            car_fs.lane_target = current_lane.id;
            car_fs.v_target = current_lane.v * deltav_safe_factor;
            car_fs.changing_lane = false;
          }
        }
        break;
      case 2:
        // Check if it's safe to switch to left lane 
        left_lane = car.get_lane_kinematics(current_lane.id - 1);
        if (this->is_lane_safe(left_lane)) {
          // TURN LEFT
          car_fs.lane_target = left_lane.id;
          car_fs.v_target = left_lane.v * deltav_change_lane; 
          car_fs.changing_lane = true;
        } else {
          // STAY but maintain a lower velocity by a safety factor
          car_fs.lane_target = current_lane.id;
          car_fs.v_target = current_lane.v * deltav_safe_factor;
          car_fs.changing_lane = false;
        }
        break;
    }
  } else {
    // Car ahead is at a safe distance, so maintain max speed in current lane
    car_fs.lane_target = current_lane.id;
    car_fs.v_target = EGO_MAX_VEL;
    car_fs.changing_lane = false;
  }
  return car_fs;
}

