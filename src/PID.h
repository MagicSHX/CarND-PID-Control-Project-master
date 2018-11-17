/*
 *  PID.cpp
 *  purpose: Uses a PID to control steering and throttle response
 *
 *  @author Bruno Guisard
 *  @version 0.1 6/23/2017
 */

#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

using namespace std::chrono;

class PID {
public:
 /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double e[3];
  double twiddle_error;

 /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double p[3];

 /*
  * Optimizer Coefficients
  */
  // constant integers control the main characteristics of the optimizer
  const int start_meas = 300;         // starts measuring error for twiddle
  const int end_meas = 700;           // ending measuring error for twiddle  
  const double twiddle_w_tol = 0.2;   // Twiddle weights tolerance
  const int max_iters = 5000;           // Maximum number of iterations
  const int num_params = 3;

  double dp[3];
  double best_p[3];
  double best_err;

  int optimizer_t;
  int twiddle_pass;
  int twiddle_iter;

  bool use_twiddle = false;
  bool is_retrial = false;

  // server
  uWS::WebSocket<uWS::SERVER> server;

 /*
  * Constructor
  */
  PID();

/*
  * Destructor.
  */
  virtual ~PID();

 /*
  * Initializes PID.
  */
  void Init(double K_p, double K_i, double K_d);

/*
  * Updates the PID error variables given cross track error.
  */
  void UpdateError(double cte);

 /*
  * Calculates the total PID error.
  */
  double TotalError();

 /*
  * Calculates the steering angle.
  */
  double CalculateOutput();

 /*
  * Calculates the throttle response
  * if we had data to measure the target speed at any point
  * this could be implemented as another PID
  * but because we don't I used a throttle that is
  * inversely proportional to steering angle and speed.
  */
  double CalculateThrottle(double speed, double steering_angle);

 /*
  * Initializes twiddle parameters.
  */
  void InitializeTwiddle();

 /*
  * Optimizes tau values (Kp, Ki, Kd).
  */
  void Twiddle();

 /*
  * Restarts simulator.
  */
  void RestartSim(uWS::WebSocket<uWS::SERVER> ws);

 /*
  * Stores server info.
  */
  void SetServer(uWS::WebSocket<uWS::SERVER> ws);
};


#endif /* PID_H */