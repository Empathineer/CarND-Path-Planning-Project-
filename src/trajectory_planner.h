/*
 * trajectory_planner.h
 *
 *  Created on: 26 Jul 2020
 *  	Author: Carissa
 */

#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

// #include "base.h"
#include "helpers.h"
#include "vehicle.h"
#include "spline.h"

struct TrajectoryPlanner {
  TrajectoryPlanner();

  Path generate_traj(EgoVehicle &ego_vehicle, const MapWaypoints map_wps);
};

#endif /* TRAJECTORY_PLANNER_H_ */
