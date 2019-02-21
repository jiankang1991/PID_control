#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;

  this->prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  this->p_error = cte;
  this->d_error = cte - this->prev_cte;
  this->i_error += cte;

  this->prev_cte = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

  double total_error = this->p_error + this->d_error + this->i_error;

  return total_error;  // TODO: Add your total error calc here!
}

double PID::CalcSteer() {

  double steer_angle = -this->Kp*this->p_error - this->Kd*this->d_error - this->Ki*this->i_error; 

  return steer_angle;


}