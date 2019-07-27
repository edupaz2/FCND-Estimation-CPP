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

## Step 3 (Scenario 8):
