#include <vector>
#include <float.h>

#ifndef PID_H
#define PID_H

using namespace std;

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
  double TotalError(double cte, bool enabledTwiddle);

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

  double prev_cte;   // Previous step of cross track error.
  bool isReset;      // Flag to reset

  /**
   * Twiddle
   */
  double total_error_sqr;
  double best_error;
  int pid_index;
  int steps;

  vector<double> dp;

  enum TwiddleState {
    Normal = 1,
    Increase = 2,
    Decrease = 3
  };
  TwiddleState state;

  void NeedTwiddle(double cte);
  void Twiddle(double cte, double total_err);

  // Helpers
  void Reset();
  void PushIndex();
  void CoefUpdate(int index, double diff);
};

#endif  // PID_H