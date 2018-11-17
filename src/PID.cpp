#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// Initialize coefficients with the values given
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	// Initialize helpful variables
	prev_cte = 0;

	i_error = 0;
	d_error = 0;
	p_error = 0;

	//previous_time = clock();
}

void PID::UpdateError(double cte) {
	//double current_time = clock();
	//dt = (current_time - previous_time)/CLOCKS_PER_SEC;
	//previous_time = current_time;

	// The p term
	p_error = cte;
	// The d term
	d_error = (cte - prev_cte);
	//d_error = cte-prev_cte;
	prev_cte = cte;
	// The i term
	i_error += cte;
	//i_error +=cte;

}

double PID::TotalError() {
	return i_error;
}