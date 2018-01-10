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
* Used Devices/Systems: Robot, OptiTrack system
  On the PC in the Lab start TrackingDaisy. Now the OptiTrack system tracks the target mounted on the robot and streams the OptiTrack data to a multicast ip-address.
  Power on the robot and be shure that the robot-internal jetson computer is running.
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected?)
* On the robot, the internal jetson computer (responsible for the encoder data) and the raspberry pi (with attached accelerometer and gyroscope) have to run.
  On the PC in the Lab running TrackingDaisy, the dongle necessary for the OptiTrack system has to be attached.

Usage
-----

* *TODO* Which ROS nodes have to be started? Provide the necessary commands
  here. Put links to the sources of the started ROS nodes.
* *TODO* Write a launch file that starts all the necessary nodes for
  demonstration
  ([roslaunch](http://wiki.ros.org/roslaunch),
  [launch file format](http://wiki.ros.org/roslaunch/XML),
  [example launch file](https://github.com/tuw-cpsg/general-ros-modules/blob/master/pioneer_teleop/launch/drive.launch). It
  is then enough to show, how to start the launch file (optional)
* *TODO* Describe parameters, if available or needed (e.g., serial port,
  modes).
* *TODO* Finally remove all the TODOs.

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
