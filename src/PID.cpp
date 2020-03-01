#include <limits>
#include <cmath>
#include <iostream>
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
  // initialize PID coefficients
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  // initialize pid errors 
  PID::d_err = 0.0;
  PID::i_err = 0.0;
  PID::p_err = 0.0;

  // init twiddling params
  set_twiddle = false;
  dp = {1.0, 1.0, 1.0};
  total_err = 0;
  step = 1;
  param_index = 2;
  best_err = std::numeric_limits<double>::max();
  // tolerance = 0.0001;
  // eval_phase = true;
  // ajust_phase = false;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  if (step == 1){
    // for correct computing d_err on start
    p_err = cte;
  }
  d_err = cte -  p_err;
  p_err = cte;
  i_err += cte;



  // while (set_twiddle && ajust_phase && step < n_ajust + n_eval){
  //   std::cout << "step: " << step << std::endl;
  //   std::cout << "total error: " << total_err << std::endl;
  //   std::cout << "best error: " << best_err << std::endl;
  // }

  // update total error only if we're past number of settle steps
  if (step % (n_ajust + n_eval) > n_ajust){
      total_err += pow(cte,2);
  }
  // last step in twiddle loop... twiddle it?
  if (set_twiddle && step < n_ajust + n_eval){
      std::cout << "step: " << step << std::endl;
      std::cout << "total error: " << total_err << std::endl;
      std::cout << "best error: " << best_err << std::endl;
      if (total_err < best_err) {
          std::cout << "improvement!" << std::endl;
          best_err = total_err;
          if (step !=  n_ajust + n_eval) {
              // don't do this if it's the first time through
              dp[param_index] *= 1.1;            
          }
          // next parameter
          param_index = (param_index + 1) % 3;
          tried_adding = tried_subtracting = false;
      }
      if (!tried_adding && !tried_subtracting) {
          // try adding dp[i] to params[i]
          AddToParameterAtIndex(param_index, dp[param_index]);
          tried_adding = true;
      }
      else if (tried_adding && !tried_subtracting) {
          // try subtracting dp[i] from params[i]
          AddToParameterAtIndex(param_index, -2 * dp[param_index]);     
          tried_subtracting = true;         
      }
      else {
          // set it back, reduce dp[i], move on to next parameter
          AddToParameterAtIndex(param_index, dp[param_index]);      
          dp[param_index] *= 0.9;
          // next parameter
          param_index = (param_index + 1) % 3;
          tried_adding = tried_subtracting = false;
      }
      total_err = 0;
      std::cout << "new parameters" << std::endl;
      std::cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;        
  }
  
  step++;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

  return - Kp * p_err - Ki * i_err - Kd * d_err;  // TODO: Add your total error calc here!
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}