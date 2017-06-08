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
  // I used the parameters given in the lesson.
  pid.Init(0.2, 0.004, 3.00);
  //pid.Init(0.0, 0.000, 0.00);
  //pid.Init(0.559343, 7.89516e-06, 11.8724);
  //pid.Init(0.16, 0.002, 20.0);
  //pid.Init(0.2, 0, 20);

  std::vector<double> dp = { 1, 1, 1 };
  double * p[] = { &pid.Kp, &pid.Kd, &pid.Ki };
  unsigned int iRun = 0;
  unsigned int iDpIndex = 0;
  unsigned int iTwiddlePhase = 0;
  double err = 0.0;
  double err_sum = 0.0;
  double best_err = 1e9;
  const unsigned int N = 100;

  std::cout << "Initial Kp: " << pid.Kp << " Kd: " << pid.Kd << " Ki: " << pid.Ki << std::endl;

  h.onMessage([&bTwiddle, 
	  &pid, &dp, &p, &iRun, &iDpIndex, &iTwiddlePhase, 
	  &err, &err_sum, &best_err, &N](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

		  /*
		  if (cte > 5)
		  {
			  iRun = 0;
			  err_sum = 0;
			  pid.ResetError();
			  reset_sim(ws);
			  bTwiddle = true;
		  }
		  */

		  steer_value = pid.SteeringValue();

		  /*
		  if (iRun >= N)
		  {
			  err_sum += pow(cte, 2);
		  }

		  if (iRun >= 2*N)
		  {
			  err = err_sum / N;

			  cout << "run: " << iRun << " ERR: " << err << endl;

			  err_sum = 0;

			  iRun = 0;

			  // bTwiddle = true;
		  }
		  else
		  {
			  ++iRun;
		  }

		  //
		  // Twiddle.
		  //
		  if (bTwiddle)
		  {
			  double sum = std::accumulate(dp.begin(), dp.end(), 0.0);

			  if (sum > 0.2)
			  {
				  bool bReset = false;

				  std::cout << "Phase: " << iTwiddlePhase << " iDpIndex: " << iDpIndex << " BEST ERR: " << best_err << " ERR: " << err << " sum: " << sum << std::endl;

				  switch (iTwiddlePhase)
				  {
				  case 0:
				  {
					  *p[iDpIndex] += dp[iDpIndex];

					  ++iTwiddlePhase;

					  bReset = true;

					  break;
				  }

				  case 1:
				  {
					  if (err < best_err)
					  {
						  best_err = err;

						  dp[iDpIndex] *= 1.1;

						  iTwiddlePhase = 0;

						  ++iDpIndex;
						  iDpIndex %= 3;
					  }
					  else
					  {
						  *p[iDpIndex] -= 2 * dp[iDpIndex];

						  ++iTwiddlePhase;

						  bReset = true;
					  }

					  break;
				  }

				  case 2:
				  {
					  if (err < best_err)
					  {
						  best_err = err;
						  dp[iDpIndex] = 1.1;
					  }
					  else
					  {
						  *p[iDpIndex] += dp[iDpIndex];
						  dp[iDpIndex] *= 0.9;
					  }

					  iTwiddlePhase = 0;

					  ++iDpIndex;
					  iDpIndex %= 3;

					  break;
				  }

				  default:
				  {
					  assert(false);
					  break;
				  }
				  }

				  std::cout << "New Kp: " << pid.Kp << " Kd: " << pid.Kd << " Ki: " << pid.Ki << std::endl;

				  // Reset simulator.
				  if (bReset)
				  {
					  iRun = 0;
					  err_sum = 0;
					  bTwiddle = false;
					  pid.ResetError();
					  reset_sim(ws);
					  return;
				  }
			  }
			  else
			  {
				  bTwiddle = false;
			  }
		  } 
		  */

		  //steer_value = fmax(-1.0, steer_value);
		  //steer_value = fmin(1.0, steer_value);

		  double K = 2.0;
		  double throttle = K * (60 - speed) - pid.Kp*pid.p_error - pid.Kd * pid.d_error - pid.Ki * pid.i_error;
		  throttle = 0.3;
		  // # The throttle speed is managed according to the steering angle.
		  // throttle = max(0.1, -0.15/0.05 * abs(steering_angle) + 0.35)

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