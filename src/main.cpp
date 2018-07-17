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
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * DONE: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Convert reference angle
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (unsigned int i = 0; i < ptsx.size(); i++)
          {
            // Shift Car reference Angle to 90 degrees
            double diff_x = ptsx[i] - px;
            double diff_y = ptsy[i] - py;
            next_x_vals.push_back(diff_x * cos(0-psi) - diff_y * sin(0-psi));
            next_y_vals.push_back(diff_x * sin(0-psi) + diff_y * cos(0-psi));
          }

          Eigen::VectorXd ptsx_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(next_x_vals.data(), next_x_vals.size());
          Eigen::VectorXd ptsy_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(next_y_vals.data(), next_y_vals.size());

          // Measure Polynomial Coefficients.
          auto coeffs = polyfit(ptsx_t, ptsy_t, 3);

          //cout << "Coeffs Size: " << coeffs.size() << endl;
          //for (unsigned int i = 0; i < coeffs.size(); i++)
          //{
          //  cout << coeffs[i] << endl;
          //}

          // Calculate cte & epsi
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);    // Since psi becomes zero we can remove that from the equation.


          const double delay = 0.1; // 100ms == 0.1s
          const double Lf = 2.67;

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          //factor in delay
          double delay_x = v*delay;
          double delay_y = 0;
          double delay_psi = -v*steer_value / Lf * delay;
          double delay_v = v + throttle_value*delay;
          double delay_cte = cte + v*sin(epsi)*delay;
          double delay_epsi = epsi-v*steer_value /Lf * delay;

          // Define the state vector
          Eigen::VectorXd state(6);
          state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;

          // Call MPC Solver
          auto vars = mpc.Solve(state, coeffs);

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          unsigned int N = (unsigned int)vars[2];
          for(unsigned int i = 3; i < 2+N; ++i)
          {
              mpc_x_vals.push_back(vars[i]);
          }
          for(unsigned int i = 3+N; i < 3+2*N; ++i)
          {
              mpc_y_vals.push_back(vars[i]);
          }

          json msgJson;
          msgJson["steering_angle"] = vars[0];
          msgJson["throttle"] = vars[1];

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
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
          this_thread::sleep_for(chrono::milliseconds(100));
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
