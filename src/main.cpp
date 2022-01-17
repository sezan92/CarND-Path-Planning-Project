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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double TARGET_SPEED = 50;
const int NUM_POINTS = 50;
const int MAX_ACCN = 10;

void set_speed(double target_speed, double current_speed_mph, double car_s, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> &next_x_vals, vector<double> &next_y_vals);
double mph_to_mps(double target_speed_mph);

class Vehicle {
  public:
  Vehicle();
  Vehicle(int lane, double x, double y, double s, double d, double yaw, double ref_vel);
  
  virtual ~Vehicle();
  
  void gen_trajectory(vector<double> &next_x_vals, 
                      vector<double> &next_y_vals);
  void set_xy(double x, double y);
  void set_sd(double s, double d);
  void set_yaw(double yaw);
  void set_ref_vel(double ref_vel);
  void set_lane(int lane);
  void speedup(double accn = 0.224);
  void slowdown(double deccn = 0.448);
  void set_previous_path(vector<double> previous_path_x, vector<double> previous_path_y);
  void set_map_waypoints(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s);
  bool change_lane_left();
  bool change_lane_right();
  bool change_lane();


  int lane;
  double x, y, s, d, ref_vel, yaw;
  vector<double> previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s;

};

Vehicle::Vehicle(){}
Vehicle::Vehicle(int lane, double x, double y, double s, double d, double yaw, double ref_vel){
  this->lane = lane;
  this-> x = x;
  this-> y = y;
  this->s = s;
  this->d = d;
  this->ref_vel = ref_vel;
  this->yaw = yaw;

}
void Vehicle::set_xy(double x, double y){
  this->x = x;
  this->y = y;
}
void Vehicle::set_sd(double s, double d){
  this->s = s;
  this->d = d;
}

void Vehicle::set_yaw(double yaw){
  this->yaw = yaw;
}

void Vehicle::set_ref_vel(double ref_vel){
  this->ref_vel = ref_vel;
}

void Vehicle::set_lane(int lane){
  this->lane = lane;
}

void Vehicle::set_previous_path(vector<double> previous_path_x, vector<double> previous_path_y){
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
}

bool Vehicle::change_lane_right(){
  if (this->lane < 2) 
  {
    this->lane += 1;
    return true;
  }
  else return false;
}
bool Vehicle::change_lane_left(){
  if (this->lane > 0)
  {
    this->lane -= 1;
    return true;
  }
  else return false;
}

bool Vehicle::change_lane()
{
  if (this->lane==0){
    return this->change_lane_right();
  }
  else if (this->lane==2){
    return this->change_lane_left();
  }
  else if (this->lane==1){
    return this->change_lane_right();
  }
  else return false;

}

void Vehicle::set_map_waypoints(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s){
    
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
                         }


void Vehicle::gen_trajectory(vector<double> &next_x_vals, 
                            vector<double> &next_y_vals){
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = this->x;
  double ref_y = this->y;

  double ref_yaw = deg2rad(this->yaw);
  int prev_size = this->previous_path_x.size();

  if (prev_size < 2){
    double prev_car_x = this->x - cos(this->yaw);
    double prev_car_y = this->y - sin(this->yaw);
    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);
  }


  else
  {
    ref_x = this->previous_path_x[prev_size - 1];
    ref_y = this->previous_path_y[prev_size - 1];

    double ref_x_prev = this->previous_path_x[prev_size - 2];
    double ref_y_prev = this->previous_path_y[prev_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(this->s + 30, (2 + 4 * this->lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp1 = getXY(this->s + 60, (2 + 4 * this->lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp2 = getXY(this->s + 90, (2 + 4 * this->lane), this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  for(int i = 0; i < this->previous_path_x.size(); i++)
  {
    next_x_vals.push_back(this->previous_path_x[i]);
    next_y_vals.push_back(this->previous_path_y[i]);
  }
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x + target_y * target_y));

  double x_add_on = 0;
  double N = (target_dist / (0.02 * this->ref_vel /2.24));
  for(int i = 0; i< NUM_POINTS - this->previous_path_x.size(); i++){
    
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

  }
}

void Vehicle::speedup(double accn){
  this->ref_vel += accn;
}

void Vehicle::slowdown(double deccn){
  this->ref_vel -= deccn;
}

Vehicle::~Vehicle() {}

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
          
          
          std::cout<<"printing ego lane:  "<<ego.lane<<std::endl;
          std::cout<<"printing ego x:  "<<ego.x<<std::endl;
          std::cout<<"printing ego y:  "<<ego.y<<std::endl;
          std::cout<<"printing ego s:  "<<ego.s<<std::endl;
          std::cout<<"printing ego d:  "<<ego.d<<std::endl;
          std::cout<<"printing ego yaw:  "<<ego.yaw<<std::endl;
          std::cout<<"printing ego ref_vel:  "<<ego.ref_vel<<std::endl;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // find ref_v to use
          for (int i =0; i < sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6]; // get the lane distance
            if (d < (2 + 4 * ego.lane + 2) &&  d > (2 + 4 * ego.lane - 2)){
              //car is in my lane using this range
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s+=((double)prev_size * 0.02 * check_speed);//not sure why

              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)){
              // lower the reference velocity 
              too_close = true;
              ego.change_lane();

              }

            }

          }

          if(too_close)
          {
            ego.slowdown();
          }
          else if (ego.ref_vel < 49.5)
          {
            ego.speedup();
          }
          ego.gen_trajectory(next_x_vals, next_y_vals);

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