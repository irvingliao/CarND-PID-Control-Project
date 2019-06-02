#include "PID.h"
#include <iostream>
#include <numeric>

using namespace std;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  prev_cte = 0;
  isReset = true;

  total_error_sqr = 0.0;
  best_error = DBL_MAX;
  steps = 0;
  pid_index = 0;
  state = Normal;

  dp.push_back(0.025);
  dp.push_back(0.0003);
  dp.push_back(0.25);

  //coeff_update = START;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  if (isReset) {
    isReset = false;
    prev_cte = cte;
  }

  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = p_error;
}

double PID::TotalError(double cte, bool enabledTwiddle) {
  /**
   * Calculate and return the total error
   */
  UpdateError(cte);
  if (enabledTwiddle) {
    NeedTwiddle(cte);
  }
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

void PID::NeedTwiddle(double cte) {
  steps += 1;
  if (steps > 200) {
    total_error_sqr += cte*cte;
  }
  if (steps == 500) {
    Twiddle(cte, total_error_sqr);
  }
}

void PID::Twiddle(double cte, double total_err) {
  double sum_dp = accumulate(dp.begin(), dp.end(), 0.0);

  if (sum_dp < 0.00001) {
    cout << "No need Twiddle, sum_dp: " << sum_dp << endl;
    return;
  }

  switch (state) {
  case Increase:
    if (total_err < best_error) {
      best_error = total_err;
      dp[pid_index] *= 1.1;
      state = Normal;
      PushIndex();

    } else {
      state = Decrease;
      double diff = -2*dp[pid_index];
      CoefUpdate(pid_index, diff);
    }
    break;

  case Decrease:
    if (total_err < best_error) {
      best_error = total_err;
      dp[pid_index] *= 1.1;
      state = Normal;
      PushIndex();

    } else {
      CoefUpdate(pid_index, dp[pid_index]);
      dp[pid_index] *= 0.9;
      state = Normal;
      PushIndex();
    }
    break;

  default:
    CoefUpdate(pid_index, dp[pid_index]);
    state = Increase;
    break;
  }

  Reset();
  cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << ", dp: (" << dp[0] << ", " << dp[1] << ", " << dp[2] << ")" << endl;
}

// Helpers

void PID::Reset() {
  isReset = true;
  steps = 0;
  prev_cte = 0;
  i_error = 0.0;
  total_error_sqr = 0.0;
}

void PID::PushIndex() {
  pid_index++;
  pid_index %= 3;
}

void PID::CoefUpdate(int index, double diff) {
  double n;
  switch (index) {
  case 0:
    n = Kp + diff;
    if (n > 0) {
      Kp = n;
    }
    else {
      Kp += diff/2;
    }
    break;

  case 1:
    n = Ki + diff;
    if (n > 0) {
      Ki = n;
    }
    else {
      Ki += diff/2;
    }
    break;

  case 2:
    n = Kd + diff;
    if (n > 0) {
      Kd = n;
    }
    else {
      Kd += diff/2;
    }
    break;

  default:
    break;
  }
}