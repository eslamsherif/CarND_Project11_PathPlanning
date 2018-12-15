#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

/* Module Properties and Periodicities */
#define PERIODICITY_MS (0.02)

/* lane properties */
#define LANE_WIDTH (4.0)
#define LANE_BEGIN(idx)  ( ( idx )       * LANE_WIDTH )
#define LANE_CENTER(idx) ( ( idx + 0.5 ) * LANE_WIDTH )
#define LANE_END(idx)    ( ( idx + 1.0 ) * LANE_WIDTH )
#define LANE_IDX(car_d)  ( (int) ( (int) car_d / (int) LANE_WIDTH ) )

#define LANE_1 (0U)
#define LANE_2 (1U)
#define LANE_3 (2U)

/* Algorithm Configurable parameters */
#define SEPERATION_GAP_M    (30.0)
#define FUTURE_PTS_CNT      (50U)
#define PRV_STAT_NRLY_EMPTY (2U)

/* Speed limits */
#define MAX_SPEED_LIMIT  (50.0)
#define IDLE_SPEED_LIMIT (49.5)
#define HIGH_SPEED_LIMIT (45.0)
#define MilePerHr_TO_MeterPerSec (2.24)
#define ACCL_STEP        (MilePerHr_TO_MeterPerSec / 10.0)
#define LOW_ACCL_STEP    (ACCL_STEP / 2.0)

/* Data structure parsing support */
#define SNSR_FSN_ID_IDX   (0U)
#define SNSR_FSN_X_IDX    (1U)
#define SNSR_FSN_Y_IDX    (2U)
#define SNSR_FSN_VX_IDX   (3U)
#define SNSR_FSN_VY_IDX   (4U)
#define SNSR_FSN_S_IDX    (5U)
#define SNSR_FSN_D_IDX    (6U)

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

struct car_properties {
  int idx;
  double x;
  double y;
  double vx;
  double vy;
  double v;
  double s;
  double d;
};

//from http://www.cplusplus.com/forum/general/97555/
//TODO: check if asc or des order needed.
bool carProp_Compare(car_properties lhs, car_properties rhs) { return lhs.s < rhs.s;}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

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
  const double max_s = 6945.554;

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /* Clean State */
          next_x_vals.clear();
          next_y_vals.clear();

          const int prev_size = previous_path_x.size();

          /* Find Initial lane based on car current pos */
          int Car_laneIdx = LANE_IDX(car_d);

          /* have a reference velocity to target */
          static double target_Velocity = 0;
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          /* Prediction phase, scan all detected objects to determine the
           * state of the enviornment.
           */

          vector <car_properties> lane1_infrontCars;
          vector <car_properties> lane2_infrontCars;
          vector <car_properties> lane3_infrontCars;

          for(int i=0; i < sensor_fusion.size(); i++) {
            car_properties temp;

            temp.idx = sensor_fusion[i][SNSR_FSN_ID_IDX];
            temp.x   = sensor_fusion[i][SNSR_FSN_X_IDX];
            temp.y   = sensor_fusion[i][SNSR_FSN_Y_IDX];
            temp.vx  = sensor_fusion[i][SNSR_FSN_VX_IDX];
            temp.vy  = sensor_fusion[i][SNSR_FSN_VY_IDX];
            temp.s   = sensor_fusion[i][SNSR_FSN_S_IDX];
            temp.d   = sensor_fusion[i][SNSR_FSN_D_IDX];

            const double target_speed  = sqrt( temp.vx * temp.vx + temp.vy * temp.vy );
            const double pred_target_s = temp.s + ( (double) prev_size * PERIODICITY_MS * target_speed );

            if( ( pred_target_s > car_s ) && ( (pred_target_s - car_s) < SEPERATION_GAP_M ) ) {
              /* Target car is predicted to be infront of us, find in which lane it should be placed. */
              switch(LANE_IDX(temp.d)) {
                case LANE_1:
                  lane1_infrontCars.push_back(temp);
                  break;
                case LANE_2:
                  lane2_infrontCars.push_back(temp);
                  break;
                case LANE_3:
                  lane3_infrontCars.push_back(temp);
                  break;
                default:
                  cout << "Fatal Error, Target Car not in possible lane";
                  break;
              }
            }
          }

          /* We know the cars infornt of us in each lane now, time to plan
           * The best action to follow. The plan should output:
           * Steering output (Which lane we should be in)
           * Velocity output (At what speed should we move)
           */
          const int lane1_car_cnt = lane1_infrontCars.size();
          const int lane2_car_cnt = lane2_infrontCars.size();
          const int lane3_car_cnt = lane3_infrontCars.size();
          int slowdown = -1;

          switch(Car_laneIdx) {
            case LANE_1:
              /* | Car |     |     |  */
              /* We can only keep on lane 1 or move to lane 2 */
              if(lane1_car_cnt == 0) {
                /* No cars in lane 1 keep with max speed */
                slowdown = false;
              }
              else if(lane2_car_cnt == 0) {
                /* Cars found in lane 1 and No cars in lane 2 switch to it with max speed */
                Car_laneIdx = LANE_2;
                slowdown = false;
              }
              else if(lane3_car_cnt == 0) {
                /* Cars found in both lane 1 and 2, check lane 3 if empty
                 * (if lane 3 is empty it may be worth it to move to lane 2 for
                 * future moves but slow down as lane 2 has cars.
                 */
                Car_laneIdx = LANE_2;
                slowdown = true;
              }
              else {
                /* All lanes are busy just reduce car speed. */
                slowdown = true;
              }
              break;
            case LANE_2:
              /* |     | Car |     |  */
              /* We can keep on lane 2 or move to lane 1 or 3 */
              if(lane2_car_cnt == 0) {
                /* No cars in lane 2 keep with max speed */
                slowdown = false;
              }
              else if(lane1_car_cnt == 0) {
                /* Cars found in lane 1 and No cars in lane 1 switch to it with max speed */
                Car_laneIdx = LANE_1;
                slowdown = false;
              }
              else if(lane3_car_cnt == 0) {
                /* Cars found in lane 1 and No cars in lane 1 switch to it with max speed */
                Car_laneIdx = LANE_3;
                slowdown = false;
              }
              else {
                /* All lanes are busy just reduce car speed. */
                slowdown = true;
              }
              break;
            case LANE_3:
              /* |     |     | Car |  */
              /* We can only keep on lane 3 or move to lane 2 */
              if(lane3_car_cnt == 0) {
                /* No cars in lane 3 keep with max speed */
                slowdown = false;
              }
              else if(lane2_car_cnt == 0) {
                /* Cars found in lane 3 and No cars in lane 2 switch to it with max speed */
                Car_laneIdx = LANE_2;
                slowdown = false;
              }
              else if(lane1_car_cnt == 0) {
                /* Cars found in both lane 3 and 2, check lane 1 if empty
                 * (if lane 1 is empty it may be worth it to move to lane 2 for
                 * future moves.
                 */
                Car_laneIdx = LANE_2;
                slowdown = true;
              }
              else {
                /* All lanes are busy just reduce car speed. */
                slowdown = true;
              }
              break;
            default:
              cout << "Fatal Error, Out Car not in possible lane";
              slowdown = true;
              break;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x   = car_x;
          double ref_y   = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double neg_ref_yaw = 0-deg2rad(car_yaw);


          if(prev_size < PRV_STAT_NRLY_EMPTY) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else {
            ref_x   = previous_path_x[prev_size - 1];
            ref_y   = previous_path_y[prev_size - 1];
            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s + (SEPERATION_GAP_M)      , LANE_CENTER(Car_laneIdx), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + (SEPERATION_GAP_M * 2.0), LANE_CENTER(Car_laneIdx), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + (SEPERATION_GAP_M * 3.0), LANE_CENTER(Car_laneIdx), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = ((shift_x * cos( neg_ref_yaw )) - (shift_y * sin( neg_ref_yaw )));
            ptsy[i] = ((shift_x * sin( neg_ref_yaw )) + (shift_y * cos( neg_ref_yaw )));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for(int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = SEPERATION_GAP_M;
          double target_y = s(target_x);
          double target_dist = distance(0.0, 0.0, target_x, target_y);

          double x_add_on = 0.0;

          for(int i=1; i <= 50 - prev_size; i++) {

            if(true == slowdown) {
              target_Velocity -= ACCL_STEP;
            }
            else {
              if(target_Velocity < HIGH_SPEED_LIMIT) {
                target_Velocity += ACCL_STEP;
              }
              else if(target_Velocity < IDLE_SPEED_LIMIT) { /* Slow down acceleration to decrease change of crossing the speed limit */
                target_Velocity += LOW_ACCL_STEP;
              }
            }

            const double N = ( target_dist / (PERIODICITY_MS * target_Velocity / MilePerHr_TO_MeterPerSec) );
            const double step_size = target_x / N;

            double x_point = x_add_on + step_size;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = ( ( x_ref * cos( ref_yaw ) ) - ( y_ref * sin( ref_yaw) ) );
            y_point = ( ( x_ref * sin( ref_yaw ) ) + ( y_ref * cos( ref_yaw) ) );

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
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
