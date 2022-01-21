#include<vector>
#include<vector>
#include<float.h>
#include "helpers.h"
#include "spline.h"
using std::vector;
using std::string;
using std::map;
const double TARGET_SPEED = 49.5;
const int NUM_POINTS = 50;
const int MAX_ACCN = 10;



class Vehicle {
  public:
  Vehicle();
  Vehicle(int lane, double x, double y, double s, double d, double yaw, double ref_vel, string state="KL");
  
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
  void set_sensor_fusion(vector<vector<double>> sensor_fusion);
  bool change_lane_left();
  bool change_lane_right();
  bool change_lane();
  bool check_car_in_lane(int lane,  double &cost, double max_distance=30);
  void set_state(string new_state);
  void get_new_lane_cost();

  int lane;
  double x, y, s, d, ref_vel, yaw;
  string state;
  vector<double> previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s;
  vector<vector<double>> sensor_fusion;
  map<int, double> lane_cost { {0, -100000.00}, {1, -100000.00} , {2, -100000.00}, {3, -100000.00}};

};

Vehicle::Vehicle(){}
Vehicle::Vehicle(int lane, double x, double y, double s, double d, double yaw, double ref_vel, string state){
  this->lane = lane;
  this-> x = x;
  this-> y = y;
  this->s = s;
  this->d = d;
  this->ref_vel = ref_vel;
  this->yaw = yaw;
  this->state = state;

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
void Vehicle::set_state(string new_state){
  this->state = new_state;
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

    this->set_state("RCL");
    return this->change_lane_right();
  }
  else if (this->lane==2){
    this->set_state("LCL");
    return this->change_lane_left();
  }
  else if (this->lane==1){
    this->set_state("RCL");
    return this->change_lane_right();
  }
  else return false;

}

void Vehicle::get_lane_cost(){
  for (map<int, double>::const_iterator it = this->lane_cost.begin(); it!=this->lane_cost.end(); ++it ){
    int key = it->first;
    double cost = it->second;
    int new_lane;
    if (key < 3)
    {
      new_lane = key;
    }
    else
    {
      new_lane = this->lane;
    }
    bool too_close = this->check_car_in_lane(new_lane, cost);
    this->lane_cost[key] = cost;  
  }
}

bool Vehicle::check_car_in_lane(int lane, double &cost, double max_distance){
  int prev_size = this->previous_path_x.size();

  for (int i =0; i < this->sensor_fusion.size(); i++){
    float d = this->sensor_fusion[i][6]; // get the lane distance
    if (d < (2 + 4 * lane + 2) &&  d > (2 + 4 * lane - 2)){
      //car is in my lane using this range
      double vx = this->sensor_fusion[i][3];
      double vy = this->sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = this->sensor_fusion[i][5];

      check_car_s+=((double)prev_size * 0.02 * check_speed);//not sure why

      if (lane == this->lane)
      {

        if ((check_car_s > this->s) && ((check_car_s - this->s) < max_distance))
        {
          cost = this->s - check_car_s;
          return true;
          
        }

      }
      else 
      {
        if (abs(check_car_s - this->s) < max_distance){
          cost = -abs(check_car_s - this->s);
          return true;
        }
      }
    }

  }
  cost = 0;
  return false;
}

void Vehicle::set_map_waypoints(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s){
    
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
                         }

void Vehicle::set_sensor_fusion(vector<vector<double>> sensor_fusion){
  this->sensor_fusion = sensor_fusion;
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
  this->set_state("KL");
  if (this->ref_vel < TARGET_SPEED) this->ref_vel += accn;
}

void Vehicle::slowdown(double deccn){
  this->ref_vel -= deccn;
}

Vehicle::~Vehicle() {}