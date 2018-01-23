Wanderer
========

Implementation of a simple wanderer taking the path with the farthest distance
(e.g., by analyzing the scan of the laser range finder).

* *TODO* Formulate the problem (wandering into the direction of the farthest
  distance) as simple decision making. What is the utility function? What is
  the decision network (random variables used, utility, decision)? Explain.
  
* actions
  
  The commands given to the robot are linear velocity in driving direction (ROS message Twist.linear.x) and angular velocity around the robot's z-axis (Twist.angular.z).
  Hence, linear velocity (lin_vel) and angular velocity (ang_vel) are the actions in the context of simple decision making. 
  
* evidence
  
  For decision-making the point-cloud provided by the Hoyuko Laserscanner acts as evidence.
  
* (next) state
  
  The actual state of the robot (x- and y-coordinate and theta) is assumed to be (0[m],0[m],0[rad]) for each iteration of the decision making process.
  The actual state is predicted by using the possible set of actions, which are chosen to be: 
  lin_vel = (0.05, 0.1)[m/s] and ang_vel = (-0.3, -0.25, ..., 0.25, 0.3)[rad/s]
  Thus, the prediction equations are:
  d_i = lin_vel_i * dt
  theta_j = ang_vel_j * dt
  x_ij = d_i * cos(theta_j)
  y_ij = d_i * sin(theta_j)
  
  Additionally, the point-cloud from the Laserscanner is predicted by using d_i as translation and theta_p_j as rotation in a transformation.
  *TODO* more detail?
  
  utility
  
  The expected utility u_ij is chosen to be the minimum of a set of distances derived from the predicted robot position (x,y)_ij and the corresponding predicted point-cloud pc_ij.
  The set of distances is calculated between (x,y)_ij and the points from pc_ij which are within a certain field of view of the robot with a chosen aperture angle of 90°.
  The best actions lin_vel_opt and ang_vel_opt correspond to the maximum expected utility u_ij_max.
  
  *TODO* P(s'|a,e)?
  
  decision network
  
  ![alt text](link)
  
* *TODO* Implement the decision making process in a ROS node. Map the
  formulated decision maker to the code (formulas/evalutions/decisions to parts
  of the code).
* *TODO* State and explain the decision maker and the implementation.

Setup
-----

* The power supply of the robot must be turned on, which powers the robots microcontroller and the onboard computer (`daisy`), 
as well as the raspberry pi (`daisy-pi`). Gyroscopes and Accelerometer should be connected and supplied with power by the Pi.
* The Development-VM must be run on a PC connected to the Wifi `Entenhausen` and should be configured for Networking to be able to `ping daisy-pi`.
* (*Optional - wanderer is independend from absolute pose, but if mapping wants to be done it is mandatory*) Start the OptiTrack system On the PC in the Lab start TrackingDaisy. Now the OptiTrack system tracks the target mounted on the robot and streams the OptiTrack data to a multicast ip-address.
* The most important thing is to get the Hokuyo laser range finder running. It must be connected to the robot (Jetson onboard computer). For details see [this tutorial](https://tuw-cpsg.github.io/tutorials/hokuyo-urg-04lx/).

To run the primary sensor for making wandering decisions you have to start the hokuyo node on the Jetson. This is done by adding following line in the launch file (the range finder is connected to port `/dev/ttyACM3`):

```xml
<node machine="robot" name="hokuyo"
	pkg="hokuyo_node" type="hokuyo_node"
	output="screen">
    <param name="_port" value="/dev/ttyACM3" />
</node>
```

Usage
-----

If we run the launch file via

```bash
roslaunch wanderer test.launch
```

we can test the running range finder via

```bash
rostopic echo /scan
```

Now with the rangefinder node running and publishing to the topic, we can start the decision making node [`dec_node.py`](scripts/dec_node.py):

```bash
rosrun wanderer dec_node.py
```

Alternatively we created the launch file `run.launch`, which already starts the decision making node for the wanderer:

```bash
roslaunch wanderer run.launch
```

### Configuration

For the wanderer node we have two main configuration switches in the `dec_node.py`:
* `VIEW_ANGLE`: defines the cone of in front of the robot which describe the region of interest (ROI) for computing the utility. Only laser range measurements, where `abs(angle) < VIEW_ANGLE` is true are used. It is located at [line 13](https://github.com/tomas-thalmann/demos_ros/blob/089596d08b1fa8fe96442fde11d6d2d5779b42dc/wanderer/scripts/dec_node.py#L13) of `dec_node.py`.
* `MAX_THRESHOLD`: defines at which point a linear velocity of 0.0 is allowed. If the maximal measured distance within the ROI is less then `MAX_THRESHOLD` the robot is allowed to turn in place. It is located at [line 15](https://github.com/tomas-thalmann/demos_ros/blob/089596d08b1fa8fe96442fde11d6d2d5779b42dc/wanderer/scripts/dec_node.py#L15) of `dec_node.py`.

> Would be nice to pass these two parameters as arguments to be able to specify them in the launch file, but due to lack of time this feature was postponed... ;-)

