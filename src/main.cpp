#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// constants
const double max_steer_angle_degree = 25;
const double mph_to_ms_factor = 0.44704;
const double Lf = 2.67;
const long long latency_in_ms = 100;
const double latency_in_s = latency_in_ms / 1000;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // global x positions of the waypoints
          vector<double> ptsx = j[1]["ptsx"];
          // global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          vector<double> ptsy = j[1]["ptsy"];
          // global x position of the vehicle
          double px = j[1]["x"];
          // global y position of the vehicle
          double py = j[1]["y"];
          // The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions (more details below)
          double psi = j[1]["psi"];
          // current velocity in mph
          double v_mph = j[1]["speed"];
          double v_ms = mph_to_ms_factor * v_mph;

          double sim_steer_angle = j[1]["steering_angle"]; // [-25, 25]
          double sim_throttle = j[1]["throttle"]; // [-1, 1]


          // The waypoints ptsx and ptsy from the simulator are given in a global coordinate system.
          // we need them in the coordinate system of the car.
          vector<double> car_waypoints_x;
          vector<double> car_waypoints_y;
          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            car_waypoints_x.push_back(dx * cos(psi) + dy * sin(psi));
            car_waypoints_y.push_back(dy * cos(psi) - dx * sin(psi));
          }

          // TODO rework initialization of eigen vectors
          // where does the 6 come from?
          double* ptrx = &car_waypoints_x[0];
          double* ptry = &car_waypoints_y[0];
          Eigen::Map<Eigen::VectorXd> car_waypoints_x_eigen(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> car_waypoints_y_eigen(ptry, 6);


          // fit 3rd order polynomials to waypoints
          // from lesson: Vehicle Models - 8. Fitting Polynomials ff
          auto coeffs = polyfit(car_waypoints_x_eigen, car_waypoints_y_eigen, 3);

          // cross track error, calculated by inserting x=0 into our polynomial
          // positive value: too far to the right, negative too far to the left
          double cte = polyeval(coeffs, 0);

          // from lesson: Vehicle Models - 10. Errors
          // positive value: to far to the left, negative too far to the right
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);

          const double x_after_latency = v_ms * latency_in_s;
          const double y_after_latency = 0;
          const double psi_after_latency = - v_ms * sim_steer_angle * latency_in_s / Lf;
          const double v_ms_after_latency = v_ms + sim_throttle * latency_in_s;
          const double cte_after_latency = cte + v_ms * sin(epsi) * latency_in_s;
          const double epsi_after_latency = epsi + psi_after_latency;

          //state << 0, 0, 0, v_ms, cte, epsi;
          state << x_after_latency, y_after_latency, psi_after_latency, v_ms_after_latency, cte_after_latency, epsi_after_latency;

         /* std::vector<double> x_vals = {state[0]};
          std::vector<double> y_vals = {state[1]};
          std::vector<double> psi_vals = {state[2]};
          std::vector<double> v_vals = {state[3]};
          std::vector<double> cte_vals = {state[4]};
          std::vector<double> epsi_vals = {state[5]};
          std::vector<double> delta_vals = {};
          std::vector<double> a_vals = {};
          */

         // std::cout<< "state--" << " x:" << px <<" y:" <<  py << " psi:" << psi <<  "cte: " << cte << " epsi: " << epsi << std::endl;
          auto vars = mpc.Solve(state, coeffs);

            /*x_vals.push_back(vars[0]);
            y_vals.push_back(vars[1]);
            psi_vals.push_back(vars[2]);
            v_vals.push_back(vars[3]);
            cte_vals.push_back(vars[4]);
            epsi_vals.push_back(vars[5]);

            delta_vals.push_back(vars[6]);
            a_vals.push_back(vars[7]);
             */
          double steer_value = -vars[0] / deg2rad( max_steer_angle_degree );
          double throttle_value = vars[1];

          std::cout << "steerangle: " << steer_value << std::endl;
          std::cout << "throttle: " << throttle_value << std::endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // TODO replace with MPC values
          for (int i = 2; i < vars.size(); i ++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // ** Display the waypoints/reference line **
          // points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 1; i < car_waypoints_x.size(); i++) {
            next_x_vals.push_back(car_waypoints_x.at(i));
            next_y_vals.push_back(car_waypoints_y.at(i));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency_in_ms));
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
