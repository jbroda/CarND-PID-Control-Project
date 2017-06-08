#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <numeric>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return std::stringstream();
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    std::stringstream tmp = std::stringstream();
    tmp.str(s.substr(b1, b2 - b1 + 1));
    return tmp;
  }
  return std::stringstream();
}

void reset_sim(uWS::WebSocket<uWS::SERVER> & ws)
{
	{
		json msgJson;
		msgJson["steering_angle"] = 0.0;
		msgJson["throttle"] = 0.0;
		std::string msg = "42[\"steer\"," + msgJson.dump() + "]";
		(ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	}

	// Reset simulator.
	{
		std::string msgReset = "42[\"reset\",{}]";
		ws.send(msgReset.data(), msgReset.length(), uWS::OpCode::TEXT);
	}
}

int main()
{
  uWS::Hub h;

  PID pid;

  bool bTwiddle = false;

  // TODO: Initialize the pid variable.
  pid.Init(0.2, 0.004, 3.00);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s.str() != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") 
		{
          // j[1] is the data JSON object
          double cte   = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
		  double steer_value = 0.0;

          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

		  pid.UpdateError(cte);

		  steer_value = pid.SteeringValue();

		  double throttle = 0.3;

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    (ws).close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}