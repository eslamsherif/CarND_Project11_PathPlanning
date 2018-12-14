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
#include <cmath>

using namespace std;

// for convenience
using json = nlohmann::json;

#define STD_OFF (0U)
#define STD_ON  (1U)

/* Test Code configurations */
#define STRAIGHT_LINE_CODE_FROM_CLASS (STD_OFF)
#define FOLLOW_LANE_CODE_FROM_CLASS   (STD_OFF)
#define SMOOTH_TRAJ_CODE_FROM_CLASS   (STD_OFF)
#define SMOOTH_TRAJ_OWN_OPTIMIZATION  (STD_ON)

/* Module Properties and Periodicities */
#define PERIODICITY_MS (0.02)
#define EMPTY_LIST (0U)
#define NRLY_EMPTY (2U)

/* lane properties */
#define LANE_WIDTH (4.0)
#define LANE_BEGIN(idx)  ( ( idx )       * LANE_WIDTH )
#define LANE_CENTER(idx) ( ( idx + 0.5 ) * LANE_WIDTH )
#define LANE_END(idx)    ( ( idx + 1.0 ) * LANE_WIDTH )
#define LANE_IDX(car_d)  ( (int) ( car_d / LANE_WIDTH ) )
#define LANE_1 (0U)
#define LANE_2 (1U)
#define LANE_3 (2U)

/* Algorithm Configurable parameters */
#define SEPERATION_GAP_M    (30.0)
#define FUTURE_PTS_CNT      (50U)
#define LOOK_AHEAD_CNT      (3U)

/* Car length is 4.8m, width is 2.5m and lane width is 4m */
/* from symmetry 2.4 (mycar) + 2.4 (threatcar) + 0.7 (minimum_safe_gap) */
#define LOG_DANGER_GAP_M     (5.5)
#define LOG_SEPERATION_GAP_M (SEPERATION_GAP_M)
/* from symmetry 1.25 (mycar) + 1.25 (threatcar) + 0.7 (minimum_safe_gap) */
#define LAT_DANGER_GAP_M (3.2)

/* Speed limits */
#define MAX_SPEED_LIMIT  (50.0)
#define IDLE_SPEED_LIMIT (49.5)
#define HIGH_SPEED_LIMIT (40.0)
#define MID_SPEED_LIMIT  (30.0)
#define LOW_SPEED_LIMIT  (20.0)
#define MlPH_TO_MrPS     (2.24)
#define MAX_ACC          (0.224)
#define LOW_ACC          (0.112)
#define MAX_DEACC        (-0.224)
#define LOW_DEACC        (-0.112)

/* Data structure parsing support */
#define SNSR_FSN_ID_IDX   (0U)
#define SNSR_FSN_X_IDX    (1U)
#define SNSR_FSN_Y_IDX    (2U)
#define SNSR_FSN_VX_IDX   (3U)
#define SNSR_FSN_VY_IDX   (4U)
#define SNSR_FSN_S_IDX    (5U)
#define SNSR_FSN_D_IDX    (6U)

#define CHECK_LANE_SLOTS(LANE_IDX, CAR_IN_LANE_LIST)                                       \
                if(CAR_IN_LANE_LIST.size() != EMPTY_LIST) {                                \
                  sort(CAR_IN_LANE_LIST.begin(), CAR_IN_LANE_LIST.end(), carProp_Compare); \
                  slowDown = true;                                                         \
                  int temp = ( fmod(CAR_IN_LANE_LIST[0].s , LOG_SEPERATION_GAP_M) ) - 1U;  \
                  for(int i = temp; i > 0U; i--) {                                         \
                    lane_open_pts[i][LANE_IDX] = true;                                     \
                  }                                                                        \
                }                                                                          \
                else {                                                                     \
                  for(int i = LOOK_AHEAD_CNT; i > 0U; i--) {                               \
                    lane_open_pts[i][LANE_IDX] = true;                                     \
                  }                                                                        \
                }

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
bool carProp_Compare(car_properties lhs, car_properties rhs) { return lhs.s < rhs.s; }

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

static int cnt = 0;
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
    ////cout << sdata << endl;
    //cout << "Test-1" << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

    //cout << "Test0" << endl;
      auto s = hasData(data);
//cout << "Test10" << endl;
      if (s != "") {
        auto j = json::parse(s);
//cout << "Test11" << endl;
        string event = j[0].get<string>();
//cout << "Test12" << endl;
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //cout << "Test1" << endl;

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

            int prev_size = previous_path_x.size();

            int lane_idx = LANE_IDX(car_d);

            #if(SMOOTH_TRAJ_OWN_OPTIMIZATION == STD_ON)
            static double target_Velocity = 0.0;
            double accel_val = 0.0;

            bool lane_open_pts[LOOK_AHEAD_CNT][LOOK_AHEAD_CNT] = {0U};
            //cout << "Test2" << endl;

            if(prev_size > 0) {
              car_s = end_path_s;
            }

            /* 1) Sort All surronding cars by distance */
            vector <car_properties> immediate_threat;
            vector <car_properties> lane1_infront;
            vector <car_properties> lane2_infront;
            vector <car_properties> lane3_infront;
            //cout << "Test3" << endl;

            for(int i=0; i < sensor_fusion.size(); i++) {
              car_properties temp;

              temp.idx = sensor_fusion[i][SNSR_FSN_ID_IDX];
              temp.x   = sensor_fusion[i][SNSR_FSN_X_IDX];
              temp.y   = sensor_fusion[i][SNSR_FSN_Y_IDX];
              temp.vx  = sensor_fusion[i][SNSR_FSN_VX_IDX];
              temp.vy  = sensor_fusion[i][SNSR_FSN_VY_IDX];
              temp.s   = sensor_fusion[i][SNSR_FSN_S_IDX];
              temp.d   = sensor_fusion[i][SNSR_FSN_D_IDX];

              double vx = sensor_fusion[i][SNSR_FSN_VX_IDX];
              double vy = sensor_fusion[i][SNSR_FSN_VY_IDX];

            /* 1.1) */
              double threat_speed = sqrt( vx*vx + vy*vy);
              int threat_car_lane_idx = LANE_IDX(temp.d);

              double threat_car_s = sensor_fusion[i][SNSR_FSN_S_IDX];
              threat_car_s += ( (double) prev_size * PERIODICITY_MS * threat_speed ); //predict threat car current S coordinates

              temp.v = threat_speed;

              double s_diff = fabs(threat_car_s - car_s);
              double d_diff = fabs(temp.d - car_d);

              //cout << "Test51 " << s_diff << " " << d_diff << endl;
              if( ( (s_diff) < LOG_DANGER_GAP_M ) && ( (d_diff) < LAT_DANGER_GAP_M ) ) {
                //cout << "IMM " << s_diff << " " << d_diff << endl;
                immediate_threat.push_back(temp);
              }
              else if ( ( threat_car_s > car_s ) && ( ( threat_car_s - car_s ) < (LOG_SEPERATION_GAP_M * LOOK_AHEAD_CNT) ) ) {
                if ( ( threat_car_lane_idx == LANE_1 ) ) {
                  lane1_infront.push_back(temp);
                }
                else if ( ( threat_car_lane_idx == LANE_2 ) ) {
                  lane2_infront.push_back(temp);
                }
                else {
                  lane3_infront.push_back(temp);
                }
              }

              /* Remaining cases is that sensed car is either far ahead of us or far behind us and this is not very important */
            }
            //cout << "Test4" << endl;
            bool slowDown = false;

            /* 2) Mark open spots infront of car       */
            switch(lane_idx) {
              case LANE_1:
                /* check lane 2 */
                //cout << "Test42" << endl;
                CHECK_LANE_SLOTS(LANE_2, lane2_infront);
                slowDown = false;
                /* check lane 1 */
                //cout << "Test41" << endl;
                CHECK_LANE_SLOTS(LANE_1, lane1_infront);
                /* lane 3 already closed */
                break;
              case LANE_2:
                /* check lane 1 */
                //cout << "Test43" << endl;
                CHECK_LANE_SLOTS(LANE_1, lane1_infront);
                /* check lane 3 */
                //cout << "Test45" << endl;
                CHECK_LANE_SLOTS(LANE_3, lane3_infront);
                slowDown = false;
                /* check lane 2 */
                //cout << "Test44" << endl;
                CHECK_LANE_SLOTS(LANE_2, lane2_infront);
                break;
              case LANE_3:
                /* lane 1 already closed */
                /* check lane 2 */
                //cout << "Test46" << endl;
                CHECK_LANE_SLOTS(LANE_2, lane2_infront);
                slowDown = false;
                /* check lane 3 */
                //cout << "Test47" << endl;
                CHECK_LANE_SLOTS(LANE_3, lane3_infront);
                break;
            }
            //cout << "Test5" << endl;

            /* 3) Program is aware of environment (cars and open spaces),it can take action to immdeiate threats first */
            if( immediate_threat.size() != EMPTY_LIST ) {
              //cout << "Test51 " << immediate_threat.size() << endl;
              accel_val = MAX_DEACC;
              sort(immediate_threat.begin(), immediate_threat.end(), carProp_Compare);

              car_properties temp = immediate_threat[0];
              if(temp.d > car_d) { /* threat is approaching from the right */
                for(int i = 2U; i < 0U; i--) { /* Search for farthest point */
                  for (int j = 0U; j > 1U; j++) { /* on the left and center lanes */
                    if(lane_open_pts[i][j] == true) {
                      /* found a suitable point to escape */
                      lane_idx = j;
                    }
                  }
                }
              }
              else { /* threat is approaching from the left */
                for(int i = 2U; i < 0U; i--) {   /* Search for farthest point */
                  for (int j = 1U; j > 0U; j--) {  /* on the right and center lane */
                    if(lane_open_pts[i][j] == true) {
                      /* found a suitable point to escape */
                      lane_idx = j;
                    }
                  }
                }
              }
            }
            else { /* No Immediate threat, optimize course for maximum speed. */
              //cout << "Test52" << endl;
              accel_val = MAX_ACC;
              /* 4) Plane to maximize car speed but prioritize staying in same lane if more than one option is open. */
              for(int i = 2U; i < 0U; i--) {   /* Search for farthest point */
                for (int j = 1U; j > 0U; j--) {  /* on the right and center lane */
                  if(lane_open_pts[i][j] == true) {
                    lane_idx = j;
                  }
                }
              }
            }
            //cout << "Test6 " << car_d << endl;

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x   = car_x;
            double ref_y   = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double neg_ref_yaw = 0-deg2rad(car_yaw);


            if(prev_size < NRLY_EMPTY) {
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
            //cout << "Test7 " << endl;

            //cout << car_s << " " << LANE_CENTER(lane_idx) << endl;
            vector<double> next_wp0 = getXY(car_s + (SEPERATION_GAP_M)      , LANE_CENTER(lane_idx), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + (SEPERATION_GAP_M * 2.0), LANE_CENTER(lane_idx), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + (SEPERATION_GAP_M * 3.0), LANE_CENTER(lane_idx), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
              //cout << "Test71 " << ptsx[i] << " " << ptsy[i] << endl;
            }
            //cout << "Test8 " << endl;

            tk::spline s;
            s.set_points(ptsx, ptsy);
            //cout << "Test9 " << endl;

            for(int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = SEPERATION_GAP_M;
            double target_y = s(target_x);
            double target_dist = distance(0.0, 0.0, target_x, target_y);


            double x_add_on = 0.0;
            //cout << "Test01 " << endl;

            for(int i=1; i <= 50 - prev_size; i++) {
              target_Velocity += accel_val;

              if(true == slowDown) {
                target_Velocity -= MAX_DEACC;
              }
              else {
                if(target_Velocity < MAX_ACC) {
                  target_Velocity = MAX_ACC; /* ensure we never completely stop on high way */
                }
                else if(target_Velocity < MID_SPEED_LIMIT) {
                  target_Velocity += MAX_ACC;
                }
                else if(target_Velocity < HIGH_SPEED_LIMIT) {
                  target_Velocity += LOW_ACC;
                }
                else if(target_Velocity > IDLE_SPEED_LIMIT) { /* ensure we never exceed high way velocity */
                  target_Velocity -= accel_val;
                }
              }
              //cout << target_Velocity << endl;

              double N = ( target_dist / (PERIODICITY_MS * target_Velocity / MlPH_TO_MrPS) );
              double step_size = target_x / N;
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
            //cout << "Test02 " << endl;
            #endif

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            //cout << "Test03 " << cnt++ << endl;
            //cout << msg.data() << endl;

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      //cout << "Test03ES " << endl;
    }
    //cout << "Test03ES1 " << endl;
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
