[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
This project consists of a c++ implementation of a Model Predictive Control (MPC) to control the steering angle and the throttle acceleration of a car using the Udacity self driving simulator.

[//]: # (Image References)

[image1]: ./images/simulator.png "Simaluator"

# Compilation
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Implementation
## The Model
The following euqations define rules for updating the prediction model at each time step.

```c
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
* ``x``, ``y`` position of the car in the coordinate system of the car
* ``psi``orientation of the car
* ``v`` speed
* ``cte``cross track error
* ``èpsi`` error with resperct to the orientation


* ``Lf`` measures the distance between the front of the vehicle and its center of gravity. 
* ``f(x)`` is the evaluation of the polynomial ``f`` at point x 
* and ``psides`` is the tangencial angle of the polynomial ``f`` evaluated at x.


## Timestep Length and Elapsed Duration (N & dt)
The prediction horizon is the duration over which predictions are made. 
* ``dt=100ms`` the same duration as the latency of the system
* ``N=8`` this amount of steps allows for observing about ``44,7m  = 100mph*0.44704mps/mph*1s = 100mph*1s ``
I reduced and increased the prediction horizon and figured out, that too many steps decreased the accuracy. 


## Polynomial Fitting and MPC Preprocessing
The simulator return the position of the car and the waypoints in a global cordinate system. Before fitting the waypoints, they have to be translated into the coordinate system of the car:

```c
for (int i = 0; i < ptsx.size(); i++) {
  double dx = ptsx[i] - px;
  double dy = ptsy[i] - py;
  car_waypoints_x.push_back(dx * cos(psi) + dy * sin(psi)); // positive values: ahead
  car_waypoints_y.push_back(dy * cos(psi) - dx * sin(psi)); // positive values: to the left
}
```
After preprocessing, the polynomial is fitted using the helper function ``polyfit`` (file main.cpp at line xxxx). 
`


## Model Predictive Control with Latency
The goal of the Model Predictive Control algorithm is to find appropriate values for the steering angle and throttle that results in an predicted trajectory which is as close as possible to the polynom that was fitted to the waypoints


![equation](http://latex.codecogs.com/gif.latex?%5Cdelta%20%5Cepsilon%20%5B-25%5E%7B%5Ccirc%7D%2C%2025%5E%7B%5Ccirc%7D%5D)

![equation](http://latex.codecogs.com/gif.latex?a%20%5Cepsilon%20%5B-1%2C%201%5D)

### Constraints
The actuators constraints limits the upper and lower bounds of the steering angle and throttle acceleration/brake.
```c
// The upper and lower limits of delta are set to -25 and 25
// degrees (values in radians).
const double steering_angle_constraint = 0.436332; // Constraint for steering angle
const double throttle_constraint = 1.0; // Constraint for acceleration

for (int i = delta_start; i < a_start; i++) {
  vars_lowerbound[i] = -steering_angle_constraint;
  vars_upperbound[i] = steering_angle_constraint;
}

// Acceleration/decceleration upper and lower limits.
// NOTE: Feel free to change this to something else.
for (int i = a_start; i < n_vars; i++) {
  vars_lowerbound[i] = -throttle_constraint;
  vars_upperbound[i] = throttle_constraint;
}
```

### Costfunction
```c
double ref_v = 40;
double ref_cte = 0;
double ref_epsi = 0;

const int cte_cost_weight = 1;
const int epsi_cost_weight = 30;
const int v_cost_weight = 1;
const int delta_cost_weight = 5;
const int a_cost_weight = 10;
const int delta_change_cost_weight = 50000;
const int jerk_cost_weight = 1;`

// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
  fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
  fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
  fg[0] += delta_change_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += jerk_cost_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

### Dealing with latency

```c
const double x_after_latency = v_ms * latency_in_s;
const double y_after_latency = 0;
const double psi_after_latency = - v_ms * sim_steer_angle * latency_in_s / Lf;
const double v_ms_after_latency = v_ms + sim_throttle * latency_in_s;
const double cte_after_latency = cte + v_ms * sin(epsi) * latency_in_s;
const double epsi_after_latency = epsi + psi_after_latency;

state << x_after_latency, y_after_latency, psi_after_latency, v_ms_after_latency, cte_after_latency, epsi_after_latency;
```
# Simulation
## The vehicle must successfully drive a lap around the track.
No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

The car can't go over the curb, but, driving on the lines before the curb is ok.


# Code Style
I tried to stick to the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
In order to check the guidelines I installed cpplint using 
`pip install cpplint`





