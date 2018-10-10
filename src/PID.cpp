#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tw_tf) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pre_cte = 0;
    p_error = 0;
    i_error = 0;
    d_error = 0;

    p.resize(3);
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    dp.resize(3);
    dp[0] = 0.05;
    dp[1] = 5.0e-5;
    dp[2] = 0.5;

    err = 0.0;
    best_err = 10e9;
    index = 0;
    cnt = 0;
    cnt_bar = 600;
    
    twiddle_tf = tw_tf;
    if (twiddle_tf) {
        p[index] += dp[index];
    }
}

void PID::UpdateError(double cte) {
    p_error = cte;
    d_error = cte - pre_cte;
    i_error += cte;
    pre_cte = cte;
}

double PID::TotalError() {
    err += pre_cte * pre_cte;
    double res = -(p_error * Kp + d_error * Kd + i_error * Ki);
    if (twiddle_tf) twiddle();
    return res;
}


void PID::twiddle() {
    cnt++;
    if (cnt == cnt_bar/2) {
        if (err < best_err) {
            best_err = err;
            dp[index] *= 1.1;
            cnt = 0;
            
            index = (index+1) % 3;
            p[index] += dp[index];
        } else {
            p[index] -= 2*dp[index];
        }
        err = 0.0;
        i_error = 0;

        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    } else if (cnt == cnt_bar) {
        if (err < best_err) {
            best_err = err;
            dp[index] *= 1.1;
        } else {
            p[index] += dp[index];
            dp[index] *= 0.9;
        }
        cnt = 0;
        index = (index+1) % 3;
        p[index] += dp[index];
        err = 0.0;
        i_error = 0.0;

        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
    }

}
