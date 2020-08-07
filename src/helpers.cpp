/*
 * helpers.cpp
 *
 *  Created on: 26 Jul 2020
 *      Author: Carissa
 */

#include "helpers.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;



string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double mph2ms(double x) { return x / 2.2369; }
double ms2mph(double x) { return x * 2.2369; }

double norm(double x, double y) {
	return sqrt(x * x + y * y);
}

double norm(double x, double y, double z) {
	return sqrt(x * x + y * y + z * z);
}

// Calculate magnitude of normal and tangential components 
double magnitude(double x, double y) {
  double magnitude = sqrt(pow(x, 2) + pow(y, 2));
  return magnitude;
}


double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Path::size(){
  return this->pts_x.size();
}

Path Path::previous_path_from_json(const nlohmann::json &j) {

  auto prev_path_x = j[1]["previous_path_x"];
  auto prev_path_y = j[1]["previous_path_y"];

  Path path;
  for(int i = 0; i < prev_path_x.size(); i++) {
    path.pts_x.push_back(prev_path_x[i]);
    path.pts_y.push_back(prev_path_y[i]);
  }

  path.end_s = j[1]["end_path_s"];
  path.end_d = j[1]["end_path_d"];

  return path;
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y - y),(map_x - x) );

  double angle = abs(theta-heading);

  if(angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}


vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};

}


vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

vector<double> get_JMT(vector<double> start, vector<double> end, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.
  INPUTS
  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]
  end   - the desired end state for vehicle. Like "start" this is a
      length three array.
  T     - The duration, in seconds, over which this maneuver should occur.
  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  EXAMPLE
  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
        3*T*T, 4*T*T*T,5*T*T*T*T,
        6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
        end[1]-(start[1]+start[2]*T),
        end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector<double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;
}


// Evaluate a polynomial.
vector<double> polyeval(vector<double> &coeffs, vector<double> &x) {
    vector<double> result(x.size());
    for (int j = 0; j < x.size(); j++) {
        result[j] = 0;
        for (int i = 0; i < coeffs.size(); i++) {
            result[j] += coeffs[i] * pow(x[j], i);
        }
    }
    return result;
}

double polyeval(vector<double> &coeffs, double x) {
    double result = 0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}


double polyeval(Eigen::VectorXd &coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

