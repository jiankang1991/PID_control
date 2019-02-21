#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  vector<double> p = {0.154872, 0.00105081, 1.69493};    // initial p vector
  vector<double> dp = {0.01, 0.0001, 0.1}; // dp vector

  bool twiddle = false;                    // whether to twiddle
  bool first_run = true;                  // whether the first vehicle loop has been run, corresponding to python code:run(robot,p)
  bool second_run = true;                 // whether the second vehicle loop has been run, corresponding to python code:run(robot,p)

  pid.Init(p[0], p[1], p[2]);
  
  int n = 0;                              // one cycle of simulator
  int n_begin = 0;                      // where to start to calculate loss
  int n_end = 700;                        // where to end to calculate loss and restart the simulator

  bool move_idx = false;                  // whether to move idx for p
  int p_idx = 0;                          // start p idx

  double total_error = 0.0;               
  double error = 0.0;
  double best_error = 999999;
  int iteration = 0;                      // iteration to record how many loops of simulator 
  double tol = 0.01;

  bool best_find = false;                 // flag of whether best p is found
  vector<double> p_best = {0.0,0.0,0.0};  // best p vector


  h.onMessage([&pid, &n, &n_begin, &n_end, &total_error, &error, &first_run, &second_run, &twiddle, &p, &p_idx, &dp, &best_find, &p_best, &best_error, &iteration, &move_idx, &tol](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          json msgJson;

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
        
          if (twiddle) {
            // if restart the simulator, initialize the pid
            if (n==0) {
              pid.Init(p[0], p[1], p[2]);
            }
            // calculate the accumulated error
            if (n>=n_begin && n<n_end) {
              total_error += cte*cte;
            }
            // update pid and calculate steer value
            pid.UpdateError(cte);
            steer_value = pid.CalcSteer();

            if (steer_value>1) {
              steer_value = 1;
            } else if(steer_value<-1) {
              steer_value = -1;
            }
            // simulator cycle add
            n += 1;
            // check whether it finishes one simulator loop
            if (n>n_end) {
              // initialize the best error if it is the first simulator loop
              if (iteration==0) {
                best_error = total_error / (n_end - n_begin);
              }
              // flag whether the p should be increased
              if (first_run) {
                p[p_idx] += dp[p_idx];
                first_run = false;
              } else
              {
                error = total_error / (n_end - n_begin);
                // whether it reduces the error and flag whether to go into the second check
                if (error < best_error && second_run) {

                  p_best[0] = p[0];
                  p_best[1] = p[1];
                  p_best[2] = p[2];

                  best_error = error;
                  dp[p_idx] *= 1.1;

                  move_idx = true;

                } else {
                  // second run has not been done, first should decrease the p
                  if (second_run) {
                    p[p_idx] -= 2*dp[p_idx];
                    second_run = false;
                  } else // if second run completes, check the error
                  {
                    if (error < best_error) {
                      p_best[0] = p[0];
                      p_best[1] = p[1];
                      p_best[2] = p[2];

                      best_error = error;
                      dp[p_idx] *= 1.1;
                      
                      move_idx = true;

                    }else
                    {
                      p[p_idx] += dp[p_idx];
                      dp[p_idx] *= 0.9;

                      move_idx = true;

                    } 
                  }
                }
              }
              // whether to move to the next idx of p, if the corresponding check has been completed
              if (move_idx) {
                p_idx += 1;
                first_run = true;
                second_run = true;
                move_idx = false;
              }

              if (p_idx==3) {
                p_idx = 0;
              }

              double sum_dp = dp[0] + dp[1] + dp[2];
              if (sum_dp < tol) {
                best_find = true;
              }

              n = 0;
              iteration += 1;
              total_error = 0.0;
              
              
              if (n==0 && !best_find) {

                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              } else if (best_find) {
                std::cout << "find best p" << std::endl;
                std::cout << "best p: " << p_best[0] << ", " << p_best[1] << ", " << p_best[2] << ", " << std::endl;
              }
            } else // end n>n_end
            {
            
               // DEBUG
              // std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
              //           << std::endl;
              if (n%100 == 0) {
                std::cout << "n: " << n << std::endl;
                std::cout << "iteration: " << iteration << std::endl;
                std::cout << "best_error: " << best_error << std::endl;
                std::cout << "current best p up till now: " << p_best[0] << ", " << p_best[1] << ", " << p_best[2] << ", " << std::endl;
              } 
              

              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            }

          }else // end twiddle
          {
            
            pid.UpdateError(cte);
            steer_value = pid.CalcSteer();

            if (steer_value>1) {
              steer_value = 1;
            } else if(steer_value<-1) {
              steer_value = -1;
            }

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                      << std::endl;

            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          

          
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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