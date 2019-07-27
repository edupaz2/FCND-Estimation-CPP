# Writeup

## Step 1 (Scenario 6):
Calculate the standard deviation of `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY`.
<p align="center">
<img src="images/1.stdev.png" width="500"/>
</p>

## Step 2 (Scenario 7):
Use a Non linear filter instead of the default linear one.
Using the equations from section 7.1.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj):
<p align="center">
<img src="images/2.1.non-linear-filter.png" width="500"/>
</p>
<p align="center">
<img src="images/2.2.non-linear-filter.png" width="500"/>
</p>
Class `Math/Quaternion` is handy for this task.

Check function `UpdateFromIMU()`: [source code](./src/QuadEstimatorEKF.cpp#L74-L124)

## Step 3 (Scenario 8):
Implement the prediction step of the filter.

### 1. Implement the transition function.
Using the equation from section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj):
<p align="center">
<img src="images/3.1.transition-function.png" width="500"/>
</p>
and knowing that the current state x_t is:
<p align="center">
<img src="images/3.2.state-x_t.png" width="500"/>
</p>

Check function `PredictState()`: [source code](./src/QuadEstimatorEKF.cpp#L149-L189)

<p align="center">
<img src="images/scenario8.1.png" width="500"/>
</p>

### 2. Calculate partial derivative of body-to-global Rotation matrix:
Using the equation from section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj):
<p align="center">
<img src="images/3.3.rbg-prime.png" width="500"/>
</p>

Check function `GetRbgPrime()`: [source code](./src/QuadEstimatorEKF.cpp#L191-L232)

### 3. Predict the state covariance forward:
Using the equation from section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj):
<p align="center">
<img src="images/3.4.covariance.png" width="500"/>
</p>

and the Predict function for EKF:
<p align="center">
<img src="images/3.5.predict.png" width="500"/>
</p>

Check function `Predict()`: [source code](./src/QuadEstimatorEKF.cpp#L234-L284)

<p align="center">
<img src="images/scenario8.1.png" width="500"/>
</p>

