
## Project: Building an Estimator

---


# Required Steps for a Passing Submission:
1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.
3. Implement all of the elements of the prediction step for the estimator.
4. Implement the magnetometer update.
5. Implement the GPS update.
6. Flight Evaluation
7. Write it up.

## [Rubric](https://review.udacity.com/#!/rubrics/1807/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup
The Drone EKF Estimator code is in C++ source file [`QuadEstimatorEKF.cpp`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/src/QuadEstimatorEKF.cpp) and its parameters for sensors are inside the text file [`QuadEstimatorEKF.txt`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/config/QuadEstimatorEKF.txt). This project also uses code from [`Controller Project`](https://github.com/etendue/FCND-Controls-CPP), i.e. source file [`QuadController.cpp`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/src/QuadControl.cpp) and parameter file text file [`QuadControlParams.txt`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/config/QuadControlParams.txt). The Controller is explained in its [`writeup.md`](https://github.com/etendue/FCND-Controls-CPP/blob/master/writeup.md).


#### Theory Behind Estimator
In this project Extended Kalman Filter(EKF) is applied to estimate Drone dynamic state. The implemented algorithm follows [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/).  I don't repeat the whole math of EKF  again in this document, but only refer these attach specific Rubic points.


#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

The measured GPS X data and Accelorometer X data are stored in text files. I used `numpy`to load the measured data and get standard deviation by calling `numpy.std()`function. 

#### 2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function. The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

The better rate gyro filter is a `Nonlinear Complementary Filter`. I followed the following update equation in function `UpdateFromIMU()`in [`QuadEstimatorEKF.cpp`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/src/QuadEstimatorEKF.cpp) line `75-127`.

Steps:

1. use the state to define a quaternion, $q_t$, from the euler angles for $\phi$, $\theta$ and $\psi$ by calling `Quaternion<float>::FromEuler123_RPY();`

2. define $dq$ to be the quaternion that consists of the measurement of the angular rates from the IMU in the body frame by calling `IntegrateBodyRate(gyro,dtIMU)`

3. Using these two, define a predicted quaternion, $\bar{q}_t$ as follows

$$\bar{q}_t = dq * q_t $$ 

4. Finally get $\bar{\theta}_t$ and $\bar{\phi}_t$ by :

$$
  \bar{\theta}_t = \bar{q}_t.Roll()\\
  \bar{\phi}_t = \bar{q}_t.Pitch()
$$


#### 3. Implement all of the elements of the prediction step for the estimator. The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.



#### 4. Implement the magnetometer update. The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

#### 5. Implement the GPS update. The estimator should correctly incorporate the GPS information to update the current state estimate.

#### 6. Flight Evaluation
##### For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.

Yes. It meets the criteria for each step.

#### The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).

The controller code was fixed with some issues, including `max_title_angle` checking and de-tune of parameter. 


#### 7. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  




You're reading it! 
