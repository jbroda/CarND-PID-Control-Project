#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
	d_error = 0;
	i_error = 0;
	p_error = 0;
	Kd = Ki = Kp = 0;
	totalError = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	ResetError();
}

void PID::UpdateError(double cte) {
	i_error += cte;
	d_error = cte - p_error;
	p_error = cte;

	totalError += pow(cte, 2);
}

double PID::TotalError() {
	return totalError;
}

double PID::SteeringValue() {
	return -(Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}
