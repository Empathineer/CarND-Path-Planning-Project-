/*
 * trajectory_generator.cpp
 *
 *  Created on: 26 July 2020
 *  	Author: Carissa
 */

#include "trajectory_planner.h"
#include "constants.h"
#include "helpers.h"


TrajectoryPlanner::TrajectoryPlanner(){}

Path TrajectoryPlanner::generate_traj(EgoVehicle &ego_vehicle,
                                  const MapWaypoints map_wps) {

  Path previous_path = ego_vehicle.prev_path;
  int prev_path_size = previous_path.size();

  // IMPORTANT:
  // In order to maintain a smoother transition from cycle to cycle, a new path is
  // generated by appending spline generated pts to the remaining pts that had not been traversed
  // from the previous path 

  // List of (x,y) reference waypoints to be interpolated by a spline.
  // More pts will be generated from this spline in order to stabilize velocity.
  vector<double> spline_pts_x, spline_pts_y;
  // first two reference waypoints
  double prev_ref_x, prev_ref_y;
  double ref_x, ref_y, ref_yaw;

  // velocity of endpts from previous path 
  double ref_vel;

  // If there are sufficient points:
  // 1) use the previous path's endpoints as starting reference to make the new path
  //    tangent to the previous one
  // 2) set the reference vel the last two points given dt = 0.02s
  
  if (prev_path_size >= 2) {
    ref_x = previous_path.pts_x[prev_path_size - 1];
    ref_y = previous_path.pts_y[prev_path_size - 1];

    prev_ref_x = previous_path.pts_x[prev_path_size - 2];
    prev_ref_y = previous_path.pts_y[prev_path_size - 2];

    ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    ref_vel = sqrt(pow(ref_x - prev_ref_x, 2) + pow(ref_y - prev_ref_y, 2)) / (double)PATH_DT ;
  }
  // If previous path is almost empty:
  // 1) use the car's actual state as the starting reference for position and velocity,
  //    setting ref_vel to the car's actual velocity
  // 2) using waypoint just behind the car such that it's oriented 
  //    tangent to the car's trajectory 
  else {
    ref_x = ego_vehicle.state.x;
    ref_y = ego_vehicle.state.y;
    ref_yaw = deg2rad(ego_vehicle.state.yaw);

    // 2nd waypoint behind the car such that the path is tangent to the car's trajectory
    // TODO: scale the cos and sin with v * dt.
    prev_ref_x = ref_x - 1.0 * cos(ref_yaw);
    prev_ref_y = ref_y - 1.0 * sin(ref_yaw);

    ref_vel = ego_vehicle.state.v;
  }

  spline_pts_x.push_back(prev_ref_x);
  spline_pts_x.push_back(ref_x);

  spline_pts_y.push_back(prev_ref_y);
  spline_pts_y.push_back(ref_y);

  // Accelerate if target speed is not achieved at the end of previous path.
  double delta_v = 0.22;
  if (ego_vehicle.fsm_state.v_target > ref_vel) {
    ref_vel += delta_v;
  }
  else {
    ref_vel -= delta_v;
  }

  // After appending the two reference points from the prev_path, set the remainder of 
  // reference waypoints for the spline, evenly spaced at 30, 60 and 90 m respectively. 
  // Therefore, 5 * (2 + 3) points are used in total. 
  double start_s = ego_vehicle.state.s;
  
  if (prev_path_size > 0) {
    start_s = previous_path.end_s;
  }
  
  for (int i = 0; i < 3; i++) {
    double wp_s = start_s + (i + 1) * 30;
    double wp_d = 2 + 4 * ego_vehicle.fsm_state.lane_target;
    vector<double> ref_wp = getXY(wp_s, wp_d, map_wps.s, map_wps.x, map_wps.y);
    spline_pts_x.push_back(ref_wp[0]);
    spline_pts_y.push_back(ref_wp[1]);
  }

  // Transform to new local coordinates:
  // 1) origin: position of car OR end point of the previous path
  // 2) orientation: car's yaw OR tangent to the ending of the previous path
  
  for (int i = 0; i < spline_pts_x.size(); i++) {
    double shift_x = spline_pts_x[i] - ref_x;
    double shift_y = spline_pts_y[i] - ref_y;

    spline_pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    spline_pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // create spline object
  tk::spline spl;

  // set spline reference points
  spl.set_points(spline_pts_x, spline_pts_y);

  // spline points will eventually be stored in pts_x and ptx_y of this struct
  Path path;

  // start by using the remainder of previous path 
  for (int i = 0; i < prev_path_size; i++) {
    path.pts_x.push_back(previous_path.pts_x[i]);
    path.pts_y.push_back(previous_path.pts_y[i]);
  }

  // break up the spline in a set of points such that the car travels at the desired velocity
  double target_x = 30;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  // fill up the rest of the path planner (up to 50 points)
  for (int i = 0; i <= 50 - prev_path_size; i++) {
    double N = target_dist / (.02 * ref_vel);
    double x_ref = (i + 1) * target_x / N;
    double y_ref = spl(x_ref);

    // switch back to global coordinates
    double x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    double y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    path.pts_x.push_back(x_point);
    path.pts_y.push_back(y_point);
  }
  return path;
}