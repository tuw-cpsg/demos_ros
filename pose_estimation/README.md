Pose Estimation
===============

Sensor fusion of pose data of our rover `daisy`.

The pose can be measured and estimated by different ROS nodes using different
sensors. For example, the OptiTrack system in our lab provides the position of
the rover very accurately (<1mm). A localization algorithm may use a map of the
lab and the laser scanner to estimate the current position. The rover can
integrate the position using the odometry information from encoders (given an
initial position).

Our rover `daisy` has many more additional sensors on board that can be
fused. This package implements sensor fusion of different pose sources on the
rover (OptiTrack is not fused, but used as reference).

Model
-----

We designed the state **__`x = `__**`[x, y, theta, omega, v, a]` of the robot as a 6-parameter state for the pose in 2d-space.
The 6 parameters are in this order:
* x-coordinate `x`
* y-coordinate `y`
* heading `theta`
* rotation `omega`
* translational velocity `v`
* translational acceleration `a`

The model is defined as:
```
x_t = x_t-1 + v_t-1 * dt * cos(theta_t-1)
x_t = x_t-1 + v_t-1 * dt * cos(theta_t-1)
theta_t = theta_t-1 + omega_t-1 * dt
omega_t = u_omega,t
v_t = v_t-1 + a_t-1 * dt
a_t = (u_v,t - v_t-1) / dt
```

As it can be seen above the two input parameters are also used, namely **__`u = `__**`[u_v, u_omega]`, which describe the control input (steering input) to the robot. 
They are provided by the ros topic `/p2os/cmd_vel` or `/teleop/cmd_vel` respectively.

Observation data
----------------

The following sensor data is integrated as observation vector **__`z`__**.
* Gyro rotational rate at z-axis (`z_gz`)
* Accelerometer acceleration at y-axis (`z_ay`) (since we suppose y-axis aligned with robot translational axis)
* Odometry translational velocity (`z_ov`)
* Odometry rotational rate about z-axis (`z_or`)

Gyro and Accelerometer data is provided by the onboard-RPi (see Setup below) via the topics `/pi/imu3000/angular_velocity` and `/pi/kxtf9/acceleration`.


Sensor fusion algorithm
-----------------------

* *TODO* What sensor fusion algorithm is used? Why?
* *TODO* State and explain the parameters and model of the algorithm.

Setup
-----

* *TODO* What devices are used? What should be powered on?
 raspberry, imu und acc vom raspberry, onboard computer, microcontroller board
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected?)
  
Additional dependency:

```
pip install scipy
```

Topics:
/pi/imu3000/angular_velocity
/pi/kxtf9/acceleration
/teleop/cmd_vel
/p2os/pose 

Usage
-----

* First start the ROS master (this example launch file combines the `sensors.launch` with the ability to teleoperate):
```
$ roslaunch pose_estimation example.launch
```
* Then start the sensor data collection and preprocessing node:
```
$ rosrun pose_estimation sensor_node.py
```
> at this version it only inter/extrapolates the observations and control inputs to one common timestamp, but can be used for smoothing and other purposes.

describe output and plan to create message.

* Finally start the pose estimation:
```
$ rosrun pose_estimation kf_node.py
```
This one subscribes to the `/cps_pe/kfobs` topic and does the sensor fusion in a KF. At this point it also publishes a String message to the the `/cps_pe/kfestimate` topic containing a csv-string of the timestamp + the six parameters of the model in the above specified order.

Alternatively you can run it also via the launch file `run.launch` which is in fact `example.launch` + the two cps_pe Nodes:

```
$ roslaunch pose_estimation run.launch
```



