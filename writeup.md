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

$$\bar{\theta}_t = \bar{q}_t.Roll()$$
$$\bar{\phi}_t = \bar{q}_t.Pitch()$$
$$\bar{\psi}_t = \bar{q}_t.Yaw()$$


#### 3. Implement all of the elements of the prediction step for the estimator. The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

The predict function for EKF is defined as following:
$$
\begin{align}
&Predict(\mu_{t-1}, \Sigma_{t-1}, u_t, \Delta_t) \\
&\bar{\mu}_t = g(u_t, \mu_{t-1})\\
&G_t = g'(u_t, x_t, \Delta t) \\
&\bar{\Sigma}_t = G_t\Sigma_{t-1}G_t^T + Q_t\\
&Return \space \bar{\mu}_t, \bar{\Sigma}_t
\end{align}
$$

In function `PredictState` in [`QuadEstimatorEKF.cpp`](https://github.com/etendue/FCND-Estimation-CPP/blob/master/src/QuadEstimatorEKF.cpp)  line `265-273` implemented $\bar{\mu}_t = g(u_t, \mu_{t-1})$; line `274` implements $\bar{\Sigma}_t = G_t\Sigma_{t-1}G_t^T + Q_t$. The function `GetRbgPrime()` from line `190-224`implements $G_t = g'(u_t, x_t, \Delta t)$. The instance variable `ekfState`and `ekfCov` are $\bar{\mu}_t, \bar{\Sigma}_t$ repectively.


The detailed calculation is from **7.2 Transition Model** in [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/).

#### 4. Implement the magnetometer update. The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

EKF update functin is defined as following:

$$
\begin{align}
&Update(\bar{\mu}_t, \bar{\Sigma}_t, z_t) \\
&H_t = h'(\bar{\mu}_t) \\
&K_t = \bar{\Sigma}_t H_t^T(H_t \bar{\Sigma}_t H_t^T + R_t)^{-1}\\
&\mu_t = \bar{\mu}_t + K_t(z_t - h(\bar{\mu}_t))\\
&\Sigma_t = (I - K_t H_t) \bar{\Sigma}_t\\
&Return \space \mu_t, \Sigma_t
\end{align}
$$

The update function ```QuadEstimatorEKF::Update(VectorXf& z, MatrixXf& H, MatrixXf& R, VectorXf& zFromX) ``` function has been provided from line `346-364`. To implement step 4, it requires to prepare the `z, H` and `zFromX` which are $z_t$, $H_t$ and $h(\bar{\mu}_t))$. 

For magnetometer update, $z_t$ has only 1 dimension which is `yaw`. The update equation refers to **7.3.2 Magnetometer** in [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/).

*(error should be the short way around, not the long way)* means the error between `z`and `zFromX` needs to be normalized between $-\pi$ and$\pi$ (see code line`331-334`).


#### 5. Implement the GPS update. The estimator should correctly incorporate the GPS information to update the current state estimate.

GPS update follows the same update function ```QuadEstimatorEKF::Update(VectorXf& z, MatrixXf& H, MatrixXf& R, VectorXf& zFromX) ```. The only difference is GPS provides 7 dimensional measure values. Therefore $z_t$, $H_t$ and $h(\bar{\mu}_t))$ are muti-dimensional. However the GPS has relative simple measure model, i.e. $H_t$ matrix is diagnal alike.  See **7.3.1 GPS** in [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/).

#### 6. Flight Evaluation
##### For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.

Yes. It meets the criteria for each step. In the whole project the parameter tuning is most time-consuming job.


#### The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).

The controller code was fixed with some issues, including `max_title_angle` checking and de-tune of parameter. 

The result.
![Senario 11](./images/square.gif)


#### 7. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


You're reading it! 
