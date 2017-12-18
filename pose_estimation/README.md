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
Odometry observations are published by the robot to the topic `/p2os/pose`.

Needed ROS topics:
* `/pi/imu3000/angular_velocity`
* `/pi/kxtf9/acceleration`
* `/teleop/cmd_vel`
* `/p2os/pose`

Sensor fusion algorithm
-----------------------

For sensor fusion we used Kalman Filter since it is an optimal (in the sense of unbiased estimates with minimal variance) for HMMs (Hidden Markov Models) with:

* linear systems
* white noise gaussian system and observation noise
* uncorrelated system and observation noise

Since we suppose sensor data to be gaussians and the [system](#model) to be sufficiently linear for small timesteps the conditions are fulfilled. 

It consists of two steps:

* Prediction
* Update

-----
**Prediction**:

```
$ x_{k|k-1} = T_{k} * x_{k-1|k-1} + B_{k} * u_{k} $
$ P_{k|k-1} = T_{k} * P_{k-1|k-1} * T_{k}^T + Q_{k} $
```

where we use the knowledge about our system (system model `T`) and the inputs `u` to predict the state `x` to the next epoch `k`. (`B` defines the link of `u` to `x`)

In addition we predict the variance-covariance matrix (VCM) `P` of the state using the system model and the system noise `Q` (see [source code](https://github.com/tomas-thalmann/demos_ros/blob/1c65f3dd64034ebd96f837e773beaaec1ed45ffe/pose_estimation/scripts/kf_node.py#L51-L52). As it can be seen modelled the input `u` as deterministic, hence they have no contribution to the VCM of the state. Otherwise a term `B_{k} * S_{k} * B_{k}^T` could be added where `S` defines the VCM of inputs.

-----

**Update**:
Specifying the relationship of state and observations with `y = H * x` the update step is computed as follows:

```
$ D_{k} = R_{k} * H_{k} * P{k|k-1} * H_{k}^T $
$ K_{k} = P_{k|k-1} * H_{k}^T * D_{k}^-1 $
$ x_{k|k} = x_{k|k-1} + K_{k} * (z_{k} - H_{k}*x_{k|k-1}) $
$ P_{k|k} = P_{k|k-1} - K_{k} * H_{k} * P_{k|k-1} $
```

where `R_{k}` specifies the measurement noise (definition can be found at [source code](https://github.com/tomas-thalmann/demos_ros/blob/1c65f3dd64034ebd96f837e773beaaec1ed45ffe/pose_estimation/scripts/kf_node.py#L65-L69). With this we can compute VCM of the innovations (difference between predicted and observed measurements) `D_{k}`. Depending on this Kalman gain `K` is computed depending on the relation of predicted state VCM and innovation VCM. Kalman gain `K` defines the update to the predicted state of the innovations to get the updated state estimates.

Update step is finalized by reducing the corresponding state VCM using K.

Setup
-----

* The power supply of the robot must be turned on, which powers the robots microcontroller and the onboard computer (`daisy`), 
as well as the raspberry pi (`daisy-pi`). Gyroscopes and Accelerometer should be connected and supplied with power by the Pi.
* The Development-VM must be run on a PC connected to the Wifi `Entenhausen` and should be configured for Networking to be able to `ping daisy-pi`.
* For settings see the launch file `run.launch`.
  
Additional python dependency on the Dev-VM:

```
pip install scipy
```


Usage
-----

### ROS master

First start the ROS master and all required nodes (this example launch file combines the `sensors.launch` with the ability to teleoperate):
```
$ roslaunch pose_estimation example.launch
```

### Sensordata

Then start the sensor data collection and preprocessing node:
```
$ rosrun pose_estimation sensor_node.py
```
[sensor_node.py](scripts/sensor_node.py)
> at this version it only inter/extrapolates the observations and control inputs to one common timestamp, but can be used for smoothing and other purposes.

It subscribes to the the above specifed topics for observations and control inputs, interpolates them to one common timestamp and publishes as csv-string to the topic
> this string message is a quick-fix and will be changed in the future. How to extract the data from this string can bee seen at [kf_node.py](https://github.com/tomas-thalmann/demos_ros/blob/ea9131fc606e2a0cddc0ad7371a2a84cd53502d8/pose_estimation/scripts/kf_node.py#L29-L34)

### Pose estimation

Finally start the pose estimation:
```
$ rosrun pose_estimation kf_node.py
```
[kf_node.py](scripts/kf_node.py)
This one subscribes to the `/cps_pe/kfobs` topic and does the sensor fusion in a KF. At this point it also publishes a String message to the the `/cps_pe/kfestimate` topic containing a csv-string of the timestamp + the six parameters of the model in the above specified order.
> again the usage of a string message is a quick-fix and will be changed in the future.

> also when updating to a ROS message it definitely makes sense to include the estimated VCM of the states `P`.

### Launch file

Alternatively you can run it also via the launch file `run.launch` which is in fact `example.launch` + the two cps_pe Nodes:

```
$ roslaunch pose_estimation run.launch
```

The resulting ROS graph should look like:

![ROS graph](docs/rosgraph.png?raw=true "ROS graph")

when issueing:

```
$ rosrun rqt_graph rqt_graph
```


