# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Project Overview
In this project, a PID-controller in C++ is built to control the steering of a car in the simulator from Udacity (which can be downloaded [here]((https://github.com/udacity/self-driving-car-sim/releases)). The parameters in the PID-controller is optimized by the twiddle algorithm.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Parameter Optimization
In the PID-controller, the steering angle is governed by three components, P (Proportional), I (Integral), and D (Derivative), of the controller. The simulator measures the cross track error (cte) of the car and passes it to the controller for the steering angle calculation.
Here is the equation:
steering angle = - (tau_p * CTE + tau_d * d(CTE)/dt + tau_i * sum(CTE))

The twiddle algorithm is implemented to find the best parameters set of tau_p, tau_d, and tau_i. First, a good initial guess for all parameters should be used to make the car driving on the lane. Here are some recods of small videos which show the effect of each P, I, D component of the controller in the simulation.

#### P Component
Left image:  P = 1.0, I = 0.0, D = 0.1

RIght image: P = 0.1, I = 0.0, D = 0.1  
<p align="center">
  <img width="250" height="200" src="./images/P1.gif">
  <img width="250" height="200" src="./images/P2.gif">
</p>

The images above show that P component helps move the car back to the lane. But if the value is too large, it would generate intense oscillations alone the lane.

####  D Component
Left image:   P = 0.1, I = 0.0, D = 0.1

Middle image: P = 0.1, I = 0.0, D = 1.0

Right image:  P = 0.1, I = 0.0, D = 5.0
<p align="center">
  <img width="250" height="200" src="./images/D1.gif">
  <img width="250" height="200" src="./images/D2.gif">
  <img width="250" height="200" src="./images/D3.gif">
</p>

D component is proportional to the derivative of CTE. It helps the car move smoothly to the correct lane without too much overshoot. If it is too large, the car would oscillate around the lane with high frequency which can be showed in the right image.

####  I Component
Left image:   P = 0.1, I = 1.0, D = 1.0

Middle image: P = 0.1, I = 0.1, D = 1.0

Right image:  P = 0.1, I = 1.0e-4, D = 1.0
<p align="center">
  <img width="250" height="200" src="./images/I1.gif">
  <img width="250" height="200" src="./images/I2.gif">
  <img width="250" height="200" src="./images/I3.gif">
</p>

I component is based on the integral term of CTE in the controller. It helps deal with the systematic bias which is from car's bad tires' alignment. In this project, the car in the simulator has very little systematic bias, so the I component coefficient should be very small. 

#### Twiddle
After finding a good initial guess for all parameters, we can use the twiddle algorithm to find the final results for them. In the twiddle algorithm, an error term ("err"), which represents the square sum of CTE, is collected for each lap. If "err" is smaller than "best_err", then the changed hyperparameter would be accepted and "best_err" would be changed to "err". If not, we proceed with another change. In each lap, only one parameter with new value would be tested. 

The twiddle algorithm can be showed at the "twiddle" function in the "PID.cpp" file.

## Optimization Results

The initial input parameters are P = 0.1, I = 1.0e-4, D = 1.0.

The result parameters are P = 0.169, I = 3.85e-4, D = 1.47