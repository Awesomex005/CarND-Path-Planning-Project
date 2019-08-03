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

int lane = 1; // the middle lane
double ref_vel = 0.0; //mph

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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size(); 

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          if(prev_size > 0){
            car_s = end_path_s; // end_path_s is basically where we will be in the future
          }

          bool too_close = false;

          //find ref_v to use
          for(int i=0; i<sensor_fusion.size(); i++){
            
            //car in my line 
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // looking for where is the target car in the future. 
              check_car_s += ((double)prev_size*.02*check_speed); 
              // check that if in the future, the target car is in front of us and close to us, within 30 meters
              if((check_car_s > car_s) && ((check_car_s - car_s)<30)){
                //ref_vel = 29.5; //mile per hour
                too_close = true;
              }
            }
          }

          if(too_close){
            ref_vel -= .224; // 5 m/s*s, which is small than the 10 m/s*s limits
          }else if(ref_vel < 49.5){
            ref_vel += .224;
          }



          /**
           * Define a path made up of (x,y) points that the car will visit
           * sequentially every .02 seconds. 
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
          vector<double> next_wp0 = getXY(car_s+30, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+50, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
            double N = (target_dist/(.02*ref_vel/2.24)); // (ref_vel/2.24) convert mile per hour to meter per second
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