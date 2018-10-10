#ifndef PID_H
#define PID_H

#include<vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double pre_cte;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  vector<double> p;
  vector<double> dp;

  bool twiddle_tf;
  double err, best_err;
  int index;
  int cnt;
  int cnt_bar;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tw_tf);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void twiddle();
};

#endif /* PID_H */
