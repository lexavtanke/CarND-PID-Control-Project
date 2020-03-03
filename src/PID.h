#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

    /*
  * Convenience function for adding amount (dp) to a PID controller parameter based on index
  */
  void AddToParameterAtIndex(int index, double amount);

 private:
   /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

    /**
   * PID Errors
   */
  double p_err;
  double i_err;
  double d_err;



  /**
  * twiddling parameters
  */
  std::vector<double> dp;
  int step, param_index;
  // number of ajustment steps 
  int n_ajust;
  // number of steps to evaluate error
  int n_eval;
  double tolerance;
  double total_err, best_err;
  bool set_twiddle;
  bool tried_adding, tried_subtracting;
};



#endif  // PID_H