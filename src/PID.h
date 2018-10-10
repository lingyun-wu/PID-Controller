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

  double pre_cte;              // previous cte
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  vector<double> p;            // parameters in array
  vector<double> dp;           // parameters optimization coefficient
 
  bool twiddle_tf;             // whether do twiddling
  double err, best_err;        // error coefficients for twiddling
  int index;                   // parameter index in twiddling
  int cnt;                     // cte input counter
  int cnt_bar;                 // counter bar for each cycle
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
  void Init(double Kp, double Ki, double Kd, bool tw_tf, vector<double> dp);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Parameters Optimization
   */ 
  void twiddle();
};

#endif /* PID_H */
