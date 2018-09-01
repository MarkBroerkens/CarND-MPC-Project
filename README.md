[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
IIn this project I implement Model Predictive Control to drive the car around the track. This time however I am not given the cross track error, I have to calculate that myself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

[//]: # (Image References)

[image1]: ./images/simulator.png "Simaluator"

# Compilation
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Implementation
## The Model
Student describes their model in detail. This includes the state, actuators and update equations.

## Timestep Length and Elapsed Duration (N & dt)
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

## Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

# Simulation
## The vehicle must successfully drive a lap around the track.
No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

The car can't go over the curb, but, driving on the lines before the curb is ok.


# Code Style
I tried to stick to the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
In order to check the guidelines I installed cpplint using 
`pip install cpplint`





