#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;


void set_speed(double target_speed, double current_speed_mph, double car_s, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> &next_x_vals, vector<double> &next_y_vals);
double mph_to_mps(double target_speed_mph);


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Vehicle ego(1, 0, 0, 0, 0, 0, 0);
  ego.set_map_waypoints(map_waypoints_x,
                        map_waypoints_y,
                        map_waypoints_s);
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          if(prev_size > 0) car_s = end_path_s;
          bool too_close = false;

          json msgJson;

          std::cout<<"Test"<<std::endl;
          ego.set_xy(car_x, car_y);
          ego.set_sd(car_s, car_d);
          ego.set_yaw(car_yaw);
          ego.set_previous_path(previous_path_x, previous_path_y);
          ego.set_sensor_fusion(sensor_fusion);
          
          
          std::cout<<"INFO: ego lane:  "<<ego.lane<<std::endl;
          std::cout<<"INFO: ego x:  "<<ego.x<<std::endl;
          std::cout<<"INFO: ego y:  "<<ego.y<<std::endl;
          std::cout<<"INFO: ego s:  "<<ego.s<<std::endl;
          std::cout<<"INFO: ego d:  "<<ego.d<<std::endl;
          std::cout<<"INFO: ego yaw:  "<<ego.yaw<<std::endl;
          std::cout<<"INFO: ego ref_vel:  "<<ego.ref_vel<<std::endl;
          std::cout<<"INFO: sensor_fusion size: "<<ego.sensor_fusion.size()<<std::endl;
          std::cout<<"INFO: state: "<<ego.state<<std::endl;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          double car_in_lane_cost = 0;
          too_close = ego.check_car_in_lane(ego.lane, car_in_lane_cost);
          if(too_close)
          {
            ego.change_lane();
            ego.slowdown();
          }
          else
          {
            ego.speedup();
          }
          ego.gen_trajectory(next_x_vals, next_y_vals);
          ego.get_new_lane_cost();
          for(auto it = ego.lane_cost.cbegin(); it != ego.lane_cost.cend(); ++it)
          {
              std::cout <<"INFO: "<< it->first << ": " << it->second << "\n";
          }

          // std::cout<<"INFO: state cost: "<<ego.lane_cost<<std::endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

double mph_to_mps(double target_speed_mph){
  return target_speed_mph * 1610 / 3600;
}


void set_speed(double target_speed_mph, double current_speed_mph, double car_s, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> &next_x_vals, vector<double> &next_y_vals){
  
  double target_speed_mps = mph_to_mps(target_speed_mph);
  double current_speed_mps = mph_to_mps(current_speed_mph);
  
  double dist_inc = target_speed_mps / NUM_POINTS;
  for (int i = 0; i < NUM_POINTS; ++i) {
    double next_s = car_s + (i + 1) * dist_inc;
    double next_d = 6;
    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }
}