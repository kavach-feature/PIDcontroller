#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
 


//Twiddle implementation
  std::ofstream debug_file;
  bool twiddle = false;
  double p[3]= {0.049,0.00028,1.63};
  double dp[3]= {0.01,0.0001,0.1};
  int n =0;
  int max_iterations = 100;
  double total_cte = 0.0;
  double error = 0.0;
  double best_error = 10000.00;
  double tolerance = 0.001;
  int p_iter =0;
  int total_iter = 0;
  int sub_move = 0;
  bool first = true;
  bool second = true;
  double best_p[3] = {p[0],p[1],p[2]};


  //debug_file.open("debug.txt",std::ofstream::out);
  if (twiddle  == true)
  {
    pid.Init(p[0],p[1],p[2]);
  }
  else
  {
     pid.Init(0.049,0.00028,1.63);
  }

// Only proportional.
  // pid.Init(1, 0.0, 0.0);

  // Only integral.
  // pid.Init(0.0, 1.0, 0.0);

  // Only differential.
   //pid.Init(0.0, 0.0, 1.0);

  h.onMessage([&pid, &p, &dp, &n, &max_iterations, &tolerance, &error, &best_error, &p_iter, &total_iter, &total_cte, &first, &sub_move, &second, &twiddle, &best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double throttle_value = 0.3;
          json msgJson;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (twiddle==true)
          {
            total_cte += pow(cte,2);
            if(n==0)
            {
              pid.Init(p[0],p[1],p[2]);

            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

           std::cout<< "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
           //std::cout<< "CTE: " << cte <<" Count: " << n << std::endl;
            n=n+1;
            if(n>max_iterations)
            {
              if(first == true) {
                std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                p[p_iter] += dp[p_iter];
                first = false;
              }
              else
              {
                error=total_cte/max_iterations;

                if(error<best_error && second == true)
                {
                  best_error = error;
                  best_p[0] = p[0];
                  best_p[1]= p[1];
                  best_p[2]= p[2];

                  dp[p_iter] *=1.1;
                  sub_move +=1;
                  std::cout << "iteration: " << total_iter << " ";
                  std::cout << "p_iterator: " << p_iter << " ";
                  std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                  std::cout << "error: " << error << " ";
                  std::cout << "best_error: " << best_error << " ";
                  std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                }
                else
                {
                  if(second==true)
                  {
                    std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    p[p_iter] -= 2 * dp[p_iter];
                    second= false;                    
                  }
                  else
                  {
                    std::cout << "iteration: " << total_iter << " ";
                    std::cout << "p_iterator: " << p_iter << " ";
                    std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
                    if(error<best_error)
                    {
                      best_error = error;
                      best_p[0] = p[0];
                      best_p[1] = p[1];
                      best_p[2] = p[2];
                      dp[p_iter] *= 1.1;
                      sub_move += 1;
                    }
                    else
                    {
                      p[p_iter] += dp[p_iter];
                      dp[p_iter] *=0.9;
                      sub_move +=1;           
                    }
                    std::cout << "error: " << error << " ";
                    std::cout << "best_error: " << best_error << " ";
                    std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                  }

                }
              }
              if(sub_move>0)
              {
                p_iter +=1;
                first = true;
                second=true;
                sub_move = 0;                
              }
              if(p_iter ==3)
              {
                p_iter =0;            
              }
              total_cte = 0.0;
              n=0;
              total_iter+=1;

              double sumdp = dp[0]+dp[1]+dp[2];

              if(sumdp<tolerance)
              {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
              }
              else
              {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
            else
            {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }
          //If no twiddle
          else
          {
             pid.UpdateError(cte);
          steer_value=pid.TotalError();
         

         /*if (steer_value < - 1.0)
          {
            steer_value =-1.0;
          }
          if(steer_value>1.0)
          {
            steer_value=1.0;
          }*/          
          /*double des_speed = 30;
          double err_speed = speed - des_speed;
          speed_pid.UpdateError(err_speed);
          throttle_value = speed_pid.TotalError();
          if(throttle_value > 1.0) throttle_value = 1.0;
          if(throttle_value < -1.0) throttle_value = -1.0;*/


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
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