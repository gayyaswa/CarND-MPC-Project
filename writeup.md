# Implementation

## The Model
The MPC implementation is based on Kinematic model discussed in the lessons and it ignores certain parameters such as tire forces gravity and mass which makes it track the vehicle faster in comparison with the complicated dynamic models. The Kinematic model equations below are from the lessons which explains each of the parameters involved in detail:

[//]: # (Image References)

[image1]: ./captures/Kinematic_Model_From_Lectures.png "Kinematic model equations"

![alt text][image1]

In the above equations:

* x, y are vehicle positons usually (lat, lon)
* ψ Heading of the vehicle
* v Velocity of the vehicle
* cte Cross Track Error
* eψ Orientation Error

Parameters x, y, ψ and v represents the state of the vehicle whereas delta(steer angle) and a(throttle) represents the control inputs which affects the state of the vehicle over time dt. Using this model we can determine the vehicle next state at time ( t+ 1) from the the current state at (t).

##  Timestep Length and Elapsed Duration (N & dt)
The N and dt values determine the prediction horizon for the model. The horizon should be usually few seconds in front of the vehicle and we would need to continuously update the trajector of the vehicle based on the actuator inputs. Also the N determines the vector of number of control inputs [δ,a] needs to be optmizied by the model and higher this number the longer it would take to find the optimized( less cost) parameters.

For the project T value choosen was **1s**
based on that Number of timesteps N was fixed at **10** and dt was **0.1s**. This value was arrived based on the discussion in walkthrough video tried other values (20, 0.05) etc but the vehicle became unstable for these values and went out of the track so decided upon using the mentioned values.

## Polynomial Fitting and MPC Preprocessing


