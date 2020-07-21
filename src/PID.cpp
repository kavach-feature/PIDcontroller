#include "PID.h"
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp= Kp_;
  Ki=Ki_;
  Kd=Kd_;

  p_error =0.0;
  d_error =0.0;
  i_error=0.0;

  prev_cte=0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  //Updating the previous error to latest CTE
  /*p_error=cte;  

  //The differential error is the difference between latest CTE and the previous error
  d_error = cte - prev_cte;
  prev_cte=cte; */

    d_error = cte - p_error;
    p_error = cte;


    //Updating the Integral error
    i_error += cte;



}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

 double total_error = (-Kp *p_error) -(Ki*i_error) -(Kd*d_error);
 return total_error;  // TODO: Add your total error calc here!
}