Pose Estimation
===============
(20 points)

Sensor fusion of pose data of a rover.

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
* *TODO* List of pose sources. Use at least OptiTrack, odometry and your
  implemented sensor fusion node (see
  task [pose_estimation](../pose_estimation/README.md)).
* *TODO* What sensor fusion algorithm used? Why?

Setup
-----

* *TODO* What devices are used? What should be powered on?
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected and how?)

Usage
-----

* *TODO* Which ROS nodes have to be started? Provide the necessary commands
  here. Put links to the sources of the started ROS nodes.
* *TODO* Write a launch file that starts all the necessary nodes for
  demonstration
  ([roslaunch](http://wiki.ros.org/roslaunch),
  [launch file format](http://wiki.ros.org/roslaunch/XML),
  [example launch file](https://github.com/tuw-cpsg/general-ros-modules/blob/master/pioneer_teleop/launch/drive.launch). It
  is then enough to show, how to start the launch file -- optional (10 bonus
  points)
* *TODO* Describe parameters, if available or needed (e.g., serial port,
  modes).
* *TODO* Finally remove all the TODOs.

```bash
$ roscore &
:
:
```

Help
----
(you can delete this section when you're done)

Finally, this repo shall include:
* This README with copy-and-paste instructions for how to start the demo (ROS
  nodes, topic redirects if necessary). In the best case, only a ROS launch
  file has to be executed. However, a list of commands is also ok.
* A `src` folder containing the implementation
  ([reference implementation](https://github.com/tuw-cpsg/sf-pkg)).
* Optional launch file (please put into a folder called `launch`).
* OptiTrack shall not be used, however you can check your result against our
  "Lab-GPS" ;)

Grading: 80 points for implementation. 20 points for the docs in this README.
