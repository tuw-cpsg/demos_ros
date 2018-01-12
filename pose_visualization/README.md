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

![screenshot](https://github.com/alexf91/demos_ros/blob/pose-visualization-dev/pose_visualization/images/screenshot.png)

The screenshot shows the result in `rviz`. The coordinate system at the
top is the `estimate` frame, which is the origin of the points that the
Kalman-Filter and the Odometer publish.
The lower coordinate system is the position of the rover. It has a constant
offset to the point published by OptiTrack.
The red arrow is the estimated position and orientation of the Kalman-Filter
and the yellow arrow is the position estimated by the odometer, as published
by `p2os`.

* List of pose sources. Use at least OptiTrack, odometry and your
  implemented sensor fusion node
  (see [pose_estimation](../pose_estimation/README.md)).

    * Absolute position from OptiTrack. We subscribe to `ground_pose`, which
      is the position of the rover in the XY plane.

    * Kalman Filter for pose estimation. The Kalman Filter from the previous
      exercise was modified s.t. it publishes the estimate as
      `PoseWithCovarianceStamped` datatype, which can be displayed in
      `rviz`.

    * Estimated position based on odometry data. This is published by
      `p2os`.

* Coordinate systems and points

    We get the absolute position of the rover from the OptiTrack system.
    The topic `/daisy/ground_pose` publishes a Pose2D object, which contains
    X- and Y-coordinates and an angle theta in a frame called `world`.

    The published pose is the rover (plus some constant offset and rotation).
    This offset and rotation is eliminated and a new frame called `rover` is
    created. `rover` has it's origin in the center of the rover. The X-axis
    points in the direction of motion of the rover.

    The initial position of the rover is the origin for a frame called `estimate`.
    This is the origin of the coordinate system used in the estimator. All
    published estimates are in the `estimate` frame.



Setup
-----

* What devices are used? What should be powered on?

On the rover, powering on all boards is sufficient. In addition to the
rover, the OptiTrack system has to be started.

* What settings do you need on the devices? (e.g., on the rover, what
  sensors have to be connected?)

On the rover we are using the gyroscope and the acceleration sensor on the Raspberry Pi and encoders on the rover's internal board, so these have to be connected and powered on.
The internal board (daisy) needs to be reset using the green switch on the rover.
Additionally, we are also using the OptiTrack tracking system, which is started
using the "Tracking Daisy" shortcut on the desktop of the OptiTrack PC.


Usage
-----

* Which ROS nodes have to be started? Provide the necessary commands
  here. Put links to the sources of the started ROS nodes.

    The  nodes can be started with multiple `*.launch` files. The instructions
    are given below.
    We use the following nodes:

    * [`p2os_driver`](http://wiki.ros.org/p2os_driver) on `daisy`
    * [`imu3000`](https://github.com/tuw-cpsg/general-ros-modules/tree/master/drivers/imu3000) and
      [`kxtf9`](https://github.com/tuw-cpsg/general-ros-modules/tree/master/drivers/kxtf9) on `daisy-pi`
    * [`pioneer_teleop`](https://github.com/tuw-cpsg/general-ros-modules/tree/master/pioneer_teleop),
      [`pose_estimation.py`](scripts/pose_estimation.py) and [`coordinatetransforms.py`](scripts/coordinatetransforms.py) on the notebook
    * [`mocap_optitrack`](http://wiki.ros.org/mocap_optitrack) on the OptiTrack PC


```bash
# Start OptiTrack node
roslaunch mocap_optitrack mocap.launch
# Start estimator, transformer and teleop input
roslaunch pose_visualization all.launch
# Start rviz with configuration files
rviz -d config/rviz_config.rviz
```

Results
-------

* State findings (accuracy and precision).

    * **Accuracy**: The estimation of the position on it's own is quite accurate
        when the rover only moves straight forwards or backwards.

        The estimation of the orientation lags behind the orientation
        reported by the OptiTrack system, especially for fast movements.
        With only a few turns, the estimated and the real orientation
        differ by several degrees, which in turn influences the accuracy
        of the position as well.

    * **Precision**: The covariance matrix continuously increases, so
        the estimated position gets less precise.


* Explain differences in accuracy (offset) and precision
  (covariance). Where do they come from? What is noisy/inaccurate (think about
  all settings - model, sensors, parameters - and the algorithm itself)? Do
  errors accumulate? Why (not)?

    Measured values consist of the true value and additional noise.
    Accuracy is the offset of the measured value from the true value, while precision
    is the uncertainty of the measured value.

    In our model, we assume that the data we get from the encoders on the wheels
    directly translate to movement of the rover. This is not entirely accurate,
    because the wheels tend to slip, especially when rover doesn't move straight forward.
    This decreases the accuracy of the estimation.

    The data we get from the gyroscope and the acceleration sensor is noisy. In addition
    to noise introduced by the sensors themselves, the movement of the rover also creates
    additional noise (slipping wheels, jumpy movement). This decreases the precision of the estimation.

    In addition to errors introduced by the sensors, our model does not match reality.
    We assume, for instance, that the rover accelerates to the targeted speed in one
    timeframe (100ms). In reality, the acceleration of the rover is limited.
