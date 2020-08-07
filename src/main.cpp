#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"
#include "vehicle.h"
#include "trajectory_planner.h"
#include "lane_tracker.h"

using namespace std;

// for convenience
using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapWaypoints map_wps;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  	
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_wps.x.push_back(x);
  	map_wps.y.push_back(y);
  	map_wps.s.push_back(s);
  	map_wps.dx.push_back(d_x);
  	map_wps.dy.push_back(d_y);
  }

  // Instantiate Vehicle object for self-driving ego_car
  EgoVehicle ego_car = EgoVehicle();

  h.onMessage([&map_wps, &ego_car]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // initialize self-driving car's state (x, y, yaw, s, d )
          ego_car.set_state_from_simulator_json(j);

          // set points using remainder of previous path and store last pts 
          ego_car.set_previous_path_from_simulator_json(j);

          // get rest of vehicles in the road, detected by sensors
          ego_car.detect_other_vehicles_from_sensor_json(j);

          cout << "---------------" << endl;

          // LaneTracker calculates the target state 
          LaneTracker lane_tracker;
          FSMState target_state = lane_tracker.get_target_state(ego_car);

          cout << "Current SDC velocity: " << ego_car.state.v << endl;
          cout << "Target SDC velocity: " << target_state.v_target << endl;
          if(target_state.changing_lane){
            cout << "Changing lane..." << endl;
          }

          ego_car.fsm_state.lane_target = target_state.lane_target;
          ego_car.fsm_state.v_target = target_state.v_target;
          ego_car.fsm_state.changing_lane = target_state.changing_lane;

          // Generating a trajectory 
          TrajectoryPlanner traj_planner;
          Path new_traj = traj_planner.generate_traj(ego_car, map_wps);

          // push generated path back to simulator
          json msgJson;
          msgJson["next_x"] = new_traj.pts_x;
          msgJson["next_y"] = new_traj.pts_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // HTTP not used but needed for the program to compile 
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}