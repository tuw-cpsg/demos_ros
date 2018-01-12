Pose Visualization
==================

Visualization of pose data of a rover with [`rviz`](http://wiki.ros.org/rviz).

The pose can be measured and estimated by different ROS nodes using different
sensors. For example, the OptiTrack system in our lab provides the position of
the rover very accurately (<1mm). A localization algorithm may use a map of the
lab and the laser scanner to estimate the current position. The rover can
integrate the position using the odometry information from encoders (given an
initial position).

However, all sources of the pose will have different covariance and may have an
offset to each other. This package shall visualize the data of different pose
sources.

* ![alt text](https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/screenshot_rviz.png)

Setup
-----

* Used Devices/Systems: Robot, OptiTrack system
  On the PC in the Lab start TrackingDaisy. Now the OptiTrack system tracks the target mounted on the robot and streams the OptiTrack data to a multicast ip-address.
  Power on the robot and be shure that the robot-internal jetson computer is running.
  
* On the robot, the internal jetson computer (responsible for the encoder data) and the raspberry pi (with attached accelerometer and gyroscope) have to run.
  On the PC in the Lab running TrackingDaisy, the dongle necessary for the OptiTrack system has to be attached.

Usage
-----

* Nodes which have to be started on the notebook:
	- keyboard (package 'pioneer_teleop'): 
	- sensor_node.py (package 'pose_estimation'): https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_estimation/scripts/sensor_node.py
	- kf_node.py (package 'pose_estimation'): https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_estimation/scripts/kf_node.py
	- mocap_node (package 'mocap_optitrack'): https://github.com/ros-drivers/mocap_optitrack/blob/master/src/mocap_node.cpp
	- odom_tf_broadcaster (package 'pose-visualization): https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/scripts/odom_tf_broadcaster.py
	- rviz with config file
  All these nodes are started with the run.launch file of the pose-visualization package.
  
* https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/launch/run.launch
  Command: 'roslaunch pose-visualization run.launch'
  
* rviz-config file, specifying which messages should be visualized: https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/config/mocap_kf.rviz  

```bash
$ roslaunch pose_visualization run.launch
:
:
```

Results
-------

* To derive measures for accuracy and precision as well as visualize the trajectories from odometry, EKF and optitrack, a python script has to be run which works on logged csv-files: https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/scripts/process_log.py
  Data from topics can be logged with "rostopic echo -b <bag-file> <topics>"
  Results for log 1
  -----------------
  Accuracy --> coordinate offsets between odometry/EKF and optitrack in the last trajectory point
  odometry
	dx: 0.137m
	dy: 0.348m
  EKF
	dx: 0.435m
	dy: 0.236m
	
  Precision --> standard deviation of the coordinates of odometry/EKF in the last trajectory point (given in the robot coordinate frame - not optitrack frame)
  odometry (standard deviations given from wheel encoders)
	sx: 0.032m
	sy: 0.032m
  EKF (Covariance matrix is initialized as identity matrix => variances of coordinates have to be scaled as 1m standard deviation in the beginning is quite high --> scale factor in process_log.py: 0.0001m^2)
	sx: 0.704m
	sy: 2.137m
  

* The accuracy is the difference between the true quantities and the estimated ones (respectively their expected values). 
  Whereas the precision is a measure for the random fluctuations of the estimated quantities around the expected values due to observation and model errors. 
  Therefore, the precision is the distribution of the estimated quantities.
  In our case accuracy is the difference between optitrack and the Kalman filter results respectively odometry results.
  And the precision is visualized with the corresponding covariance matrices.
  Two features of the observation and model errors have to be looked at (regarding precision and accuracy):
  
  Distribution --> Precision
  --------------------------
  As we use an extended Kalman filter (EKF), the observation and model errors have to be gaussian. 
  In this case also the distribution of the estimated quantities is gaussian (if the non-linearities are neglictible).
  Otherwise the estimated quantities can be biased (accuracy!) and the corresponding distribution will not capture the real one.
  The observations from accelerometer, gyroscope and odometry as well as the keyboard inputs are noisy which is accounted for by specifying the corresponding covariance matrices in the EKF.
  
  Expected Value --> Accuracy
  ---------------------------
  The expected values of observation and model errors are assumed to be zero.
  In this case the expected value coincides with the real value.
  If the expected values are non-zero, systematic offsets to the reality are present.
  In our case this can be:
	- not considered accelerometer parameters (e.g. bias) --> accumulative error (every EKF step a not considered bias is added)
	- not considered gyroscope parameters (e.g. bias) --> accumulative error
    - not considered odometry parameters (e.g. bias) --> accumulative error
	- inaccurate description of the system equation in the EKF (robot model) --> accumulative error
	- inaccurate realization of the reference (e.g. offset between optitrack target and robot reference point used in EKF) --> constant error
