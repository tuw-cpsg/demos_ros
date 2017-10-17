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

* *TODO* Fill out additional documentation, i.e., (at least) answer the
  questions below.
* *TODO* Any changes or additions to the description above?
* *TODO* List of sources (sensors/data) you use for pose fusion.
* *TODO* What sensor fusion algorithm is used? Why?
* *TODO* State and explain the parameters and model of the algorithm.

Setup
-----

* *TODO* What devices are used? What should be powered on?
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected?)

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


(you can delete everything below and all todos when you're done)

Organizational Notes
--------------------

Information and tutorials of our rovers can be found on
our [group's GitHub page](https://tuw-cpsg.github.io/).

Available sensors / pose sources:
* pose published by p2os based on encoders
  ([p2os_driver](http://wiki.ros.org/p2os_driver)); when you start the node,
  the robot is assumed to be at position (0,0); the encoders are used to
  integrate the position;
* gyroscope IMU-3000
  ([driver](https://github.com/tuw-cpsg/general-ros-modules/))
* accelerometer KXTF9
  ([driver](https://github.com/tuw-cpsg/general-ros-modules/))
* pose published by acml based on particle filtering of laser range
  measurements with a known map
  ([hokuyo_node](http://wiki.ros.org/hokuyo_node),
  [acml](http://wiki.ros.org/amcl), [gmapping](http://wiki.ros.org/gmapping))
* OptiTrack shall not be used, however, you can check your result against our
  "Lab-GPS" ;)

All nodes publishing sensor data are provided. A launch file to start the nodes
is provided in this package (`launch/sensors.launch`).

Finally, this repo shall include:
* A `src` folder containing the implementation
  ([ROS tutorials](http://wiki.ros.org/ROS/Tutorials),
  [getting started with Eigen](http://eigen.tuxfamily.org/dox/GettingStarted.html),
  [reference implementation](https://github.com/tuw-cpsg/sf-pkg)).
* This README with answered questions and copy-and-paste instructions for how
  to start the demo (ROS nodes, topic redirects if necessary). In the best
  case, only a ROS launch file has to be executed. However, a list of commands
  is also ok.
* Optional launch file (please put into a folder called `launch`).

### Grading

| Points |                     |
|-------:|---------------------|
|     80 | implementation      |
|     20 | docs in this README |
|    +10 | launch file         |
