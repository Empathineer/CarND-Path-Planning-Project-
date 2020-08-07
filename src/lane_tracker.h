/*
 * lane_tracker.h
 *
 *  Created on: 27 Jul 2020
 *      Author: Carissa Chan
 */

#ifndef LANE_TRACKER_H_
#define LANE_TRACKER_H_

// #include "base.h"
#include "helpers.h"
#include "vehicle.h"

using namespace std;



struct LaneTracker {
    LaneTracker();
    bool is_lane_safe(LaneKinematics lane_kinematics);
    FSMState get_target_state(EgoVehicle &ego_vehicle);
    int getLane(const double d, const double laneWidth);
    double getLaneOffsetD(const int lane_number, const double laneWidth);

};

#endif /* LANE_TRACKER_H */