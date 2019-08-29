#ifndef PID_H
#define PID_H
#include <iostream>
#include <math.h>
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

  void ResetCTEerror();

  void TwiddleInit();

  bool Twiddle(double tol);

  private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * PID diff Coefficients
   */
  double dKp;
  double dKi;
  double dKd;

  /**
   * PID cte errors
   */
  double cte_error;
  double best_error;

  /**
   * Twiddle parameters
   */
  unsigned int param_idx = 0; // 0: p value, 1: i value, 2: d value
  unsigned int twiddle_lvl = 0; // 0: if-state, 1: else-state
  double best_Kp;
  double best_Ki;
  double best_Kd;
};

#endif  // PID_H
