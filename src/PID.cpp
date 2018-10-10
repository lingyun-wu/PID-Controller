#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tw_tf, vector<double> dp) {
    // Initialize all variables
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pre_cte = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;

    p.resize(3);           // initialize parameters array
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    this->dp = dp;         // initialize parameter adjustment array

    err = 0.0;             // error value in twiddling
    best_err = 10e9;       // best error in twiddling
    index = 0;             // index indicates which parameter to optimize
    cnt = 0;               // count number for cte input
    cnt_bar = 2000;        // counting bar, which indicates number of inputs in a cycle
    
    twiddle_tf = tw_tf;    // boolean value indicates whether to adjust parameters
    if (twiddle_tf) {
        p[index] += dp[index];
    }
}

// Update cte, cte differential, cte integral for each input
void PID::UpdateError(double cte) {
    p_error = cte;
    d_error = cte - pre_cte;
    i_error += cte;
    pre_cte = cte;
}

// get new steering value and do parameters optimization
double PID::TotalError() {
    err += pre_cte * pre_cte;
    double res = -(p_error * Kp + d_error * Kd + i_error * Ki);
    if (twiddle_tf) twiddle();
    return res;
}


// Twiddling
void PID::twiddle() {
    cnt++;                                      // count number
    if (cnt == cnt_bar/2) {                     // one cycle for p[i] + dp[i]
        if (err < best_err) {                   // if pass
            // update errors
            best_err = err;
            dp[index] *= 1.1;
            cnt = 0;
            
            index = (index+1) % 3;              // change to next parameter   
            p[index] += dp[index];
        } else {
            p[index] -= 2*dp[index];            // if not pass, test p[i] - dp[i]
        }
        err = 0.0;
        i_error = 0;                            // cte integral reset to 0.0

        // update parameters
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    } else if (cnt == cnt_bar) {
        if (err < best_err) {                   // if p[i] - dp[i] pass
            best_err = err;                     // update errors
            dp[index] *= 1.1;
        } else {                                // if not
            p[index] += dp[index];              // change p[i] back  
            dp[index] *= 0.9;                
        }
        cnt = 0;
        index = (index+1) % 3;
        p[index] += dp[index];
        err = 0.0;
        i_error = 0.0;

        // update parameters
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    }

}
