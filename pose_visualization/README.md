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

* *TODO* Fill out additional documentation, i.e., (at least) answer the
  questions below.
* *TODO* Any changes or additions to the description above? How did you
  visualize the differences in accuracy and precision?
* *TODO* Put a
  [screenshot](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#images) of
  the result into this README.
* *TODO* List of pose sources. Use at least OptiTrack, odometry and your
  implemented sensor fusion node
  (see [pose_estimation](../pose_estimation/README.md)).
* *TODO* Unfortunately, there is
  no [display type](http://wiki.ros.org/rviz/DisplayTypes) to visualize the
  covariance of the pose in rviz directly. There are some workarounds, for
  example
  [using markers](https://geus.wordpress.com/2011/09/15/how-to-represent-a-3d-normal-function-with-ros-rviz/) (optional). However,
  it is fine if you display it in a [plot](http://wiki.ros.org/rqt_plot) or as
  an appropriate output on the console (as long as we can follow the
  differences in covariance somehow, it is ok).

Setup
-----

* *TODO* What devices are used? What should be powered on?

  Used Devices/Systems: Robot, OptiTrack system
  On the PC in the Lab start TrackingDaisy. Now the OptiTrack system tracks the target mounted on the robot and streams the OptiTrack data to a multicast ip-address.
  Power on the robot and be shure that the robot-internal jetson computer is running.
  
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected?)
  
  On the robot, the internal jetson computer (responsible for the encoder data) and the raspberry pi (with attached accelerometer and gyroscope) have to run.
  On the PC in the Lab running TrackingDaisy, the dongle necessary for the OptiTrack system has to be attached.

Usage
-----

* *TODO* Which ROS nodes have to be started? Provide the necessary commands
  here. Put links to the sources of the started ROS nodes.
  
  Nodes which have to be started on the notebook:
	- keyboard (package 'pioneer_teleop'): 
	- sensor_node.py (package 'pose_estimation'): https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_estimation/scripts/sensor_node.py
	- kf_node.py (package 'pose_estimation'): https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_estimation/scripts/kf_node.py
	- mocap_node (package 'mocap_optitrack'): https://github.com/ros-drivers/mocap_optitrack/blob/master/src/mocap_node.cpp
	- odom_tf_broadcaster (package 'pose-visualization): SOURCE
  All these nodes are started with the run.launch file of the pose-visualization package.
  
* *TODO* Write a launch file that starts all the necessary nodes for
  demonstration
  ([roslaunch](http://wiki.ros.org/roslaunch),
  [launch file format](http://wiki.ros.org/roslaunch/XML),
  [example launch file](https://github.com/tuw-cpsg/general-ros-modules/blob/master/pioneer_teleop/launch/drive.launch). It
  is then enough to show, how to start the launch file (optional)
  
  https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/launch/run.launch
  Command: 'roslaunch pose-visualization run.launch'
  
* *TODO* Describe parameters, if available or needed (e.g., serial port,
  modes).

  rviz-config file, specifying which messages should be visualized: https://github.com/tomas-thalmann/demos_ros/blob/pose-visualization-gr2/pose_visualization/config/mocap_kf.rviz  

```bash
$ roscore &
:
:
```

Results
-------

* *TODO* State findings (accuracy and precision).
* *TODO* Explain differences in accuracy (offset) and precision
  (covariance). Where do they come from? What is noisy/inaccurate (think about
  all settings - model, sensors, parameters - and the algorithm itself)? Do
  errors accumulate? Why (not)?
  
  The accuracy is the difference between the true quantities and the estimated ones (respectively their expected values). 
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
	- inaccurate realization of the reference (e.g. offset between optitrack target and turning point) --> constant error
  
  
(you can delete everything below and all todos when you're done)

Organizational Notes
--------------------

Finally, this repo shall include:
* This README with copy-and-paste instructions for how to start the demo (ROS
  nodes, topic redirects if necessary). In the best case, only a ROS launch
  file has to be executed. However, a list of commands is also ok.
* Config file for `rviz`. `rviz` can be started with a config file, see `rviz`
  usage (please put into a folder called `config`).
* Optional launch file (please put into a folder called `launch`).

### Grading

| Points |                              |
|-------:|------------------------------|
|     50 | visualization                |
|     50 | docs in this README          |
|    +20 | covariance in rviz           |
