#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
// what we need to use spline.h for is smoothing out that 
// kind of disjointed path that the car was following in the simulator
// try to make the car move smooth, not violate its acceleration and jerk.
#include "spline.h" 

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define MAX_SPEED_MPH 49.5//49.5
#define ACC_LIMIT 10 // m/s*s
#define DESIRE_ACC 5 // m/s*s
#define SIMULATOR_DT 0.02 // The simulator car will visit each (x, Y) path point sequentially every .02 seconds.
#define DESIRE_DELTA_SPEED mps2mph(DESIRE_ACC*SIMULATOR_DT)
#define TL -1
#define TR +1
#define CL_BEHIND_BUFFER 10 //m
#define CL_AHEAD_BUFFER 30 //m
int lane = 1; // the middle lane
double ref_vel = 0.0; //mph

bool find_car_ahead(double car_s, int &cur_lane, const vector<vector<double>> &sensor_fusion, double &ahead_s, double &speed_mph){
  bool found = false;
  double ahead_closest_s = 9999;
  double targe_car_s = 0;
  double vx = 0;
  double vy = 0;
  for(auto targe_car:sensor_fusion){
    float d = targe_car[6];
    targe_car_s = targe_car[5];
    if(d < (2+4*cur_lane+2) && d > (2+4*cur_lane-2) && targe_car_s > car_s ){
      if(targe_car_s < ahead_closest_s){
        ahead_closest_s = targe_car_s;
        vx = targe_car[3];
        vy = targe_car[4];
        found = true;
      }
    }
  }
  
  ahead_s = ahead_closest_s;
  speed_mph = mps2mph(sqrt(vx*vx + vy*vy));
  return found;
}

bool find_car_behind(double car_s, int &cur_lane, const vector<vector<double>> &sensor_fusion, double &behind_s, double &speed_mph){
  bool found = false;
  double behind_closest_s = -1;
  double targe_car_s = 0;
  double vx = 0;
  double vy = 0;
  for(auto targe_car:sensor_fusion){
    float d = targe_car[6];
    targe_car_s = targe_car[5];
    if(d < (2+4*cur_lane+2) && d > (2+4*cur_lane-2) && targe_car_s < car_s ){
      if(targe_car_s > behind_closest_s){
        behind_closest_s = targe_car_s;
        vx = targe_car[3];
        vy = targe_car[4];
        found = true;
      }
    }
  }
  
  behind_s = behind_closest_s;
  speed_mph = mps2mph(sqrt(vx*vx + vy*vy));
  return found;
}


void gen_KL_ref_v(double car_s, double &ref_vel, int &cur_lane, const vector<vector<double>> &sensor_fusion, bool &CL_req){

  double ahead_car_s = 9999;
  double ahead_car_speed_mph = 0;
  double dist_2_ahead_car = -1;
  bool car_ahead = false;
  car_ahead = find_car_ahead(car_s, cur_lane, sensor_fusion, ahead_car_s, ahead_car_speed_mph);
  
  if(car_ahead){
    dist_2_ahead_car = ahead_car_s - car_s;

    // keep distance with ahead car in between 15 to 30 meters
    if(dist_2_ahead_car < 30 && ref_vel > ahead_car_speed_mph){
      if((ref_vel - ahead_car_speed_mph) > DESIRE_DELTA_SPEED){
        ref_vel -= DESIRE_DELTA_SPEED;
        //printf("DEC ahead_car_speed_mph %f \t ref_vel %f\n", ahead_car_speed_mph, ref_vel);
      }else{
        ref_vel = ahead_car_speed_mph; 
      }
    }

    if(dist_2_ahead_car > 15 && ahead_car_speed_mph > ref_vel && ref_vel < MAX_SPEED_MPH){
      if((ahead_car_speed_mph - ref_vel) > DESIRE_DELTA_SPEED*.8){
        ref_vel += DESIRE_DELTA_SPEED*.8;
        //printf("INC ahead_car_speed_mph %f \t ref_vel %f\n", ahead_car_speed_mph, ref_vel);
      }else{
        ref_vel = ahead_car_speed_mph; 
      }
    }

    // too close
    if(dist_2_ahead_car < 15 && ref_vel > ahead_car_speed_mph*.8){
      if((ref_vel - ahead_car_speed_mph*.8) > DESIRE_DELTA_SPEED){
        ref_vel -= DESIRE_DELTA_SPEED;
      }else{
        ref_vel = ahead_car_speed_mph*.8; 
      }
    }


    if(dist_2_ahead_car > 40 && ref_vel < MAX_SPEED_MPH){
      ref_vel += DESIRE_DELTA_SPEED;
    }

    if (dist_2_ahead_car < 40 && ref_vel < MAX_SPEED_MPH){
      CL_req = true;
      //printf("KL ref_vel %f \t dist_2_ahead_car %f\n", ref_vel, dist_2_ahead_car);
    }
  }else if(ref_vel < MAX_SPEED_MPH){
      ref_vel += DESIRE_DELTA_SPEED;
  }
}

bool possible_to_CL(int cur_lane, int action, double car_s, double future_car_s, double remain_path_size,
                   const vector<vector<double>> &sensor_fusion, double &ahead_free_s, double &ahead_car_speed_mph){
  int next_lane = cur_lane + action;
  if(next_lane < 0 || next_lane > 2)
    return false;
  
  // checking car behind us
  double target_car_s = 0;
  double speed_mph = 0;
  double target_car_future_s = 0;
  double s_to_target_car = 0;
  double found = false;

  found = find_car_behind(car_s, next_lane, sensor_fusion, target_car_s, speed_mph);
  if(found){
    target_car_future_s = target_car_s + mph2mps(speed_mph) * remain_path_size * SIMULATOR_DT;
    s_to_target_car = future_car_s - target_car_future_s; // when this is negtive it means the target car will drive corss us, not possible to CL. 
    printf("try to turn %c, in the future s_to_target_car BEHIND %f \n", (action>0?'R':'L'), s_to_target_car);
    if(s_to_target_car < CL_BEHIND_BUFFER){
      printf("Not meet CL_BEHIND_BUFFER %dm reqierment.\n", CL_BEHIND_BUFFER);
      return false;
    }
  }
  
  found = false;
  found = find_car_ahead(car_s, next_lane, sensor_fusion, target_car_s, speed_mph);
  if(found){
    target_car_future_s = target_car_s + mph2mps(speed_mph) * remain_path_size * SIMULATOR_DT;
    s_to_target_car = target_car_future_s - future_car_s;
    printf("try to turn %c, in the future s_to_target_car AHEAD %f \n", (action>0?'R':'L'), s_to_target_car);
    if(s_to_target_car < CL_AHEAD_BUFFER){
      printf("Not meet CL_AHEAD_BUFFER %dm reqierment.\n", CL_AHEAD_BUFFER);
      return false;
    }
  }

  if(found){
    ahead_free_s = s_to_target_car;
    ahead_car_speed_mph = speed_mph;
  }else{
    ahead_free_s = 9999;
    ahead_car_speed_mph = 9999;
  }

  return true;
}

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
    float d_x; //dx, dy is just that normal component to the waypiont. dx, dy只是这个wp的垂直分量。
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          // The path that did not eaten by the car
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size(); 
          double future_car_s = car_s;

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          if(prev_size > 0){
            future_car_s = end_path_s; // end_path_s is basically where we will be in the future
          }

          /*
           * Stage 1
           * Determine if we need to change lane (lane), 
           * and what speed (ref_vel) we need to keep.
           */

          bool CL_req = false;

          gen_KL_ref_v(car_s, ref_vel, lane, sensor_fusion, CL_req);

          if(CL_req){
            bool good_2_tl = false;
            bool good_2_tr = false;
            double tl_free_s_dist = 0;
            double tr_free_s_dist = 0;
            double tl_ahead_car_speed_mph = 0;
            double tr_ahead_car_speed_mph = 0;
            int action = 0;

            good_2_tl = possible_to_CL(lane, TL, car_s, future_car_s, prev_size, sensor_fusion, tl_free_s_dist, tl_ahead_car_speed_mph);
            good_2_tr = possible_to_CL(lane, TR, car_s, future_car_s, prev_size, sensor_fusion, tr_free_s_dist, tr_ahead_car_speed_mph);

            if(good_2_tl || good_2_tr){
              printf("Good to CL\n");
              if(good_2_tl && good_2_tr){
                printf("CL, both line are good to go\n");
                // compare free s in next 5 sec.
                if( (tl_free_s_dist+5*mph2mps(tl_ahead_car_speed_mph)) >= 
                (tr_free_s_dist+5*mph2mps(tr_ahead_car_speed_mph)) ){
                  good_2_tr = false;
                }else{
                  good_2_tl = false;
                }
              }

              if(good_2_tl){
                action = TL;
              }else{
                action = TR;
              }
              lane  = lane + action;
              gen_KL_ref_v(car_s, ref_vel, lane, sensor_fusion, CL_req);
              printf("CL next lane %d \n", lane);
            }
          }
          
          //printf("left pre size： %d\n", prev_size);

          /**
           * Define a path made up of (x,y) points that the car will visit
           * sequentially every .02 seconds. (the simulatore defined so.)
           * The car moves 50 times a second.
           * The map's waypoint are measured from the double yellow line in the middle of the road.
           * Each lane is 4 meters wide, so in the center of the middle lane, the d value will be 1.5 * 4 = 6.
           */
          /** Since the car moves 50 times a second, 
           *  a distance of 0.5m per move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.
           */
          //double dist_inc = 0.5; // 0.5 meter. 
#if 0
          // move straight
          for (int i = 0; i < 50; ++i) {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
#endif
#if 0
          // move along the center of the middle lane
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s + (i+1)*dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
#endif
          /*
           * Stage 2
           * Use spline to generate trajectory according to the (lane) and (ref_vel) we just found.
           */

          // Create a list of widely spaced(x,y) waypoints, evenly spaced at 30m
          // Later we will interoplate these waypoints with a spline and fill it in with more points that contorl the speed.
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the current car state as starting reference.
          if(prev_size < 2){
            //Use two points that make the 'tangent path' to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // else use the previous path's end point as starting reference
          else{
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet adding evenly 30m spaced points ahead of the starting reference
          // major 'reference points' to generate new path/spline.
          vector<double> next_wp0 = getXY(future_car_s+30, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(future_car_s+50, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(future_car_s+90, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transform from global map coordinate to car coordinate, make the math calculating more easier
          // This was used to be used in MPC project.
          // making the ref point state as the orignal point in the car coordinate
          for(int i=0; i<ptsx.size(); i++){
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;

          // Set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from last time
          // the next_x_vals, the path plan output, will contain the previous left path and the new path given by spline.
          // the new path is started by the end of the previous left path.
          for(int i=0;i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Caculate how to break up the spline so that we can travel at our desired reference velocity
          double target_x = 30.0; // in car coordinate
          double target_y = s(target_x);
          // there will be a triangle (0,0) (target_x) (target_x,target_y)
          // the spline segment is inside the triangle, the length of the spline segment is longer than the target_dist. 
          double target_dist = distance(target_x, target_y, 0, 0); 

          double x_add_on = 0;

          // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for(int i=1;i<= 50-previous_path_x.size(); i++){

            // approximately calculate how we should break up the spline/path to maintain the desired reference velocity 
            // the spline maybe a curve, so here it would only be a approximation to split the spline into N segements.
            double N = (target_dist/(.02*mph2mps(ref_vel)));
            double x_point = x_add_on + (target_x)/N; // find the x value of the start of each path segement.
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // transform from car coordinate back to global map coordinate
            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }



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