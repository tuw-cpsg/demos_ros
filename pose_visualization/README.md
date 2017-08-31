Pose Visualization
==================
(10 points)

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
* *TODO* Any changes or additions to the description above?
* *TODO* Put here
  a
  [screenshot](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#images) of
  the result.
* *TODO* List of pose sources. Use at least OptiTrack, odometry and your
  implemented sensor fusion node (see
  task [pose_estimation](../pose_estimation/README.md)).

Setup
-----
(10 points)

* *TODO* What devices are used? What should be powered on?
* *TODO* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected and how?)

Usage
-----
(10 points)

* *TODO* Which ROS nodes have to be started? Provide the necessary commands
  here. Put links to the sources of the started ROS nodes.
* *TODO* Write a launch file that starts all the necessary nodes for
  demonstration
  ([roslaunch](http://wiki.ros.org/roslaunch),
  [launch file format](http://wiki.ros.org/roslaunch/XML),
  [example launch file](https://github.com/tuw-cpsg/general-ros-modules/blob/master/pioneer_teleop/launch/drive.launch). It
  is then enough to show, how to start the launch file -- optional (20 bonus
  points)
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
(20 points)

* *TODO* State findings (accuray and precision).
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
|    +10 | launch file                  |
|    +10 | video (screencast and rover) |
