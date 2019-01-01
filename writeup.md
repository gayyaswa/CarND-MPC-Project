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

Parameters x, y, ψ and v represents the state of the vehicle whereas δ(steer angle) and a(throttle) represents the control inputs which affects the state of the vehicle over time dt. Using this model we can determine the vehicle next state at time ( t+ 1) from the the current state at (t).

##  Timestep Length and Elapsed Duration (N & dt)
The N and dt values determine the prediction horizon for the model. The horizon should be usually few seconds in front of the vehicle and we would need to continuously update the trajector of the vehicle based on the actuator inputs. Also the N determines the vector of number of control inputs [δ,a] needs to be optmizied by the model and higher this number the longer it would take to find the optimized( less cost) parameters.

For the project T value choosen was **1s**
based on that Number of timesteps N was fixed at **10** and dt was **0.1s**. This value was arrived based on the discussion in walkthrough video tried other values **(20, 0.05)** etc but the vehicle became unstable for these values and went out of the track so decided upon using the mentioned values.

## Polynomial Fitting and MPC Preprocessing
Input waypoints or transformed into vehicle coordinates and transformed coordinates are used to fit a third order polynomial line. The genrated polynomial is used to compute vehicle **cte0** and **eψ0** for the initial state. The current state is passed into the cost optmizer which would geneate optimize actuation output steer angle and throttle by minimizing the **cte and  eψ**

## Model Predictive Control with Latency

Initial tunning of MPC was done with latency set to 0, I was able to make the vehicle complete the track by influencing ( X 100 ) the cost minimizer on cte and epsi. But one the actuation delay **100ms** was added vehicle crashed by going off the track. As suggested by the community I used the kinematic model and computed the initial by accouting dt ( 100 ms) delay.

``` c++
//Update the current state by accounting the 100ms delay using
//following kinematic model equations.
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// ctet+1 = ctet + vt ∗sin(eψt)∗dt
//eψt+1 = ψt − ψdest +(Vt/Lf * δt ∗dt)

double x_t1 = xt0 + ( v * cos(psi0)  * dt );
double y_t1 = yt0 + ( v * sin(psi0) * dt );
double psi_t1 = psi0 - ( v/Lf * steer_value * dt );
double v_t1 = v + throttle_value * dt;
double cte_t1 = cte0 + v * sin(epsi0) * dt;
double epsi_t1 = epsi0 - ( v/Lf * atan(coeffs[1]) * dt );

curr_state << x_t1, y_t1, psi_t1, v_t1, cte_t1, epsi_t1;
```

I also had to cost part affecting the cte , eψ and steer values by multipliers **(750, 750 and 10000)** respectively to make the vehicle complete the track. The cost part affecting the steering output has to be increased by **10000** in order to achieve smooth sequential steer values or else the vehicle was wobbling around the corners

``` c++

fg[0] += 750 * CppAD::pow(vars[cte_start + t], 2);
fg[0] += 750 * CppAD::pow(vars[epsi_start + t], 2);

fg[0] += 100000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
fg[0] += 3000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);

```

