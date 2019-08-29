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
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  dKp = 1.0 * pow(10,-1);
  dKi = 1.0 * pow(10,-4);
  dKd = 1.0 * pow(10,-1);

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  cte_error = 0.0;
  best_error = 999999999;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  cte_error += (cte*cte);
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return-1* (p_error*Kp +
	     d_error*Kd +
	     i_error*Ki);  // TODO: Add your total error calc here!
}

void PID::ResetCTEerror() {
  cte_error = 0.0;
}

void PID::TwiddleInit() {
  if (param_idx == 0){
    Kp += dKp;
  } else if (param_idx == 1) {
    Ki += dKi;
  } else if (param_idx == 2) {
    Kd += dKd;
  }
}

bool PID::Twiddle(double tol) {
  bool init_twiddle;
  bool best_found = false;
  if (param_idx == 0) {
    std::cout << "Tuning P value" << std::endl;
  } else if (param_idx == 1) {
    std::cout << "Tuning I value" << std::endl;
  } else if (param_idx == 2) {
    std::cout << "Tuning D value" << std::endl;
  }
  std::cout << "P value: " << Kp << " dP value: " << dKp << std::endl;
  std::cout << "I value: " << Ki << " dI value: " << dKi << std::endl;
  std::cout << "D value: " << Kd << " dD value: " << dKd << std::endl;

  if ((dKp + dKi + dKd) > tol) {
    if (cte_error < best_error && twiddle_lvl == 0) {
      best_found = true;
      best_error = cte_error;
      best_Kp = Kp;
      best_Kd = Kd;
      best_Ki = Ki;

      if (param_idx == 0) {
        dKp *= 1.1;
      } else if (param_idx == 1) {
	dKi *= 1.1;
      } else {
	dKd *= 1.1;
      }
      std::cout << "param index: " << param_idx << ", multiply by 1.1" << std::endl;
      param_idx = (param_idx + 1) % 3;
      init_twiddle = false;
    }
    else if (cte_error >= best_error && twiddle_lvl == 0) {
      if (param_idx == 0) {
	Kp -= 2 * dKp;
      } else if (param_idx == 1) {
	Ki -= 2 * dKi;
      } else if (param_idx == 2) {
	Kd -= 2 * dKd;
      }
      twiddle_lvl = 1;
      std::cout << "param index: " << param_idx << ", multiple by -2" << std::endl;
      init_twiddle = true;
    }
    else if (twiddle_lvl == 1) {
      if (cte_error < best_error) {
	best_found = true;
        best_error = cte_error;
	best_Kp = Kp;
	best_Kd = Kd;
	best_Ki = Ki;

	if (param_idx == 0) {
	  dKp *= 1.1;
	} else if (param_idx == 1) {
	  dKi *= 1.1;
	} else if (param_idx == 2) {
	  dKd *= 1.1;
	}
	std::cout << "param index: " << param_idx << ", multiply by 1.1" << std::endl;
	init_twiddle = false;
      }
      else {
	if (param_idx == 0) {
	  Kp += dKp;
	  dKp *= 0.9;
	} else if (param_idx == 1) {
	  Ki += dKi;
	  dKi *= 0.9;
	} else if (param_idx == 2) {
	  Kd += dKd;
	  dKd *= 0.9;
	}
	std::cout << "param index: " << param_idx << ", multiply by 0.9" << std::endl;
	init_twiddle = false;
      }
      param_idx = (param_idx + 1) % 3;
      twiddle_lvl = 0;
    }
    std::cout << "cte error: " << cte_error << std::endl;
    if (best_found == true) {
      std::cout << "************************* BEST VALUE FOUND *************************" << std::endl;
    }
    std::cout << "       " << std::endl;
    return init_twiddle;
  }
  else {
    std::cout << "TWIDDLE DONE!" << std::endl;
    exit(1);
  }
}
