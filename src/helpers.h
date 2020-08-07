  
/*
 * helper.h
 *
 *  Created on: 26 Jul 2020
 *      Author: Carissa
 */


#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"

using namespace std;


// for convenience
using std::string;
using std::vector;


// #define DT_SIM 0.02;
// #define LANE_WIDTH 4.0;
// #define ROAD_MAX_VEL 21.87; // in m/s,  2.24 mph = 1 m/s

// # define deb(x) cout << #x << " " << endl; //DEBUGGING TOOL 


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);


constexpr double pi();

double deg2rad(double x);
double rad2deg(double x);

// Calculate magnitude of normal and tangential components 
double magnitude(double x, double y);
double distance(double x1, double y1, double x2, double y2);


double norm(double x, double y);
double norm(double x, double y, double z);


// double mph2mps(double mph);
double mph2ms(double x);
double mps2mph(double mps);


struct MapWaypoints {
  vector<double> x;
  vector<double> y;
  vector<double> s;
  vector<double> dx;
  vector<double> dy;
};

// Holds the state of the vehicle in terms of its lane
// and kinematic variables. 
struct State {
  double x;
  double y;
  double yaw;
  double s;
  double d;
  double v;
  int lane;
};

// Holds the finite state parameters 
struct FSMState {
  string state;
  double s_target;
  double d_target;
  int lane_target;
  double v_target;
  bool changing_lane = false;
};

// Stores coordinates for generated path and endpts of previous one. 
struct Path {
  vector<double> pts_x;
  vector<double> pts_y;
  double end_s; //stores last tangential Frenet coord from prev_path
  double end_d; //stores last normal Frenet coord from prev_path

  int size();
  static Path previous_path_from_json(const nlohmann::json &j);
};

// Keeps track of other vehicle id's w.r.t to the main car  
struct LaneKinematics {
  int id;
  double gap_ahead;
  double gap_behind;
  double v;
};



// Path functions
int size(); //returns size of Path 
Path previous_path_from_json(const nlohmann::json &j);

//Lane functions 
int getLane(const double d, const double laneWidth);
double getLaneOffsetD(const int lane_number, const double laneWidth);


// Helper functions related to waypoints and converting from XY to Frenet or vice versa

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s);


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);

/******SET OF FUNCTIONS FOR INTERPOLATION*****/

// Evaluate a polynomial.
vector<double> polyeval(vector<double> &coeffs, vector<double> &x);

double polyeval(vector<double> &coeffs, double x);

double polyeval(Eigen::VectorXd &coeffs, double x);




#endif  // HELPERS_H