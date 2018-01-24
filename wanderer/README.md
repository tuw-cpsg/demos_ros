Wanderer
========

Implementation of a simple wanderer taking the path with the farthest distance
(e.g., by analyzing the scan of the laser range finder).

### Actions
  
  The commands given to the robot are linear velocity in driving direction (ROS message Twist.linear.x) and angular velocity around the robot's z-axis (Twist.angular.z).
  Hence, linear velocity (lin_vel) and angular velocity (ang_vel) are the actions in the context of simple decision making. We used two discrete sets of actions, defined by:
  
```
lin_vel = (0.05, 0.1, 0.2) [m/s]
ang_vel = (-0.3, -0.25, ..., 0.25, 0.3)[rad/s]
```
  
  Publishing twist messages containing actions to the topic `/teleop`:
  
  ```
  # store the commands and publish to the topic
  self.ang_vel = candidate[1]
  self.lin_vel = candidate[0]
  twist = Twist()
  twist.linear.x = self.lin_vel
  twist.angular.z = self.ang_vel
  self.pub.publish(twist)
  ```
  
### Evidence
  
  For decision-making the point-cloud provided by the Hoyuko Laserscanner acts as evidence.
  
  Code to provide Laserscanner data as numpy array containing the ranges with corresponding angles can be found at [Source Code](https://github.com/tomas-thalmann/demos_ros/blob/488bf9709a4082d36054ce90d9219ad6fb7243cd/wanderer/scripts/dec_node.py#L198-L227).
  
### Next State
 
We do the decision making on a local basis at the robot fixed coordinate system, so the actual state of the robot (x- and y-coordinate and theta) is assumed to be `(0[m],0[m],0[rad])` for each iteration of the decision making process.
  The actual state of the robot is predicted using the model from `pose_estimation` by using the possible set of actions, defined at section [Actions](#actions).
  
  Thus, the prediction equations are ([Source Code](https://github.com/tomas-thalmann/demos_ros/blob/488bf9709a4082d36054ce90d9219ad6fb7243cd/wanderer/scripts/dec_node.py#L186-L196)):
  ```
  d_i = lin_vel_i * dt
  theta_j = ang_vel_j * dt
  x_ij = d_i * cos(theta_j)
  y_ij = d_i * sin(theta_j)
  ```
  
  Additionally we added the point cloud in front of the robot (the environment) to the state, relevant for decision making. So these points from the laser range finder is predicted by using d_i as translation and theta_p_j as rotation in a transformation. This means we predict the environment (walls and obstacles) as it will be seen by the robot at the next epoch depending on the actions (see [Source Code](https://github.com/tomas-thalmann/demos_ros/blob/488bf9709a4082d36054ce90d9219ad6fb7243cd/wanderer/scripts/dec_node.py#L56-L92)).
  
  
  
### Utility
  
  The expected utility `u_ij` is chosen to be the minimum of a set of distances derived from the predicted robot position `(x,y, theta)_ij` and the corresponding predicted point-cloud `pc_ij`.
  The set of distances is calculated between `(x,y)_ij` and the points from `pc_ij` which are within a certain field of view of the robot with a chosen aperture angle of `2*VIEW_ANGLE` (see [Configuration](#configuration)).
  The best actions `lin_vel_opt` and `ang_vel_opt` correspond to the maximum expected utility `u_ij_max`.
  
  Additionally, a check on the maximum distance in each set of distances is made to avoid that the robot get stuck (e.g. in room corners) and to force him to move towards corners if far away
  If these maximum distances are all smaller than a threshold `MAX_THRESHOLD` (see [Configuration](#configuration)), `lin_vel = 0m/s` is added to the possible actions.
  
  `P(s'|a,e)` is assumed to be equal for each computed next state and therefore can be unconsidered.  
  
  Implementation can be found at [lines 121-184](https://github.com/tomas-thalmann/demos_ros/blob/488bf9709a4082d36054ce90d9219ad6fb7243cd/wanderer/scripts/dec_node.py#L121-L184).
  
### Decision Network
  
  ![alt text](https://github.com/tomas-thalmann/demos_ros/blob/wanderer-gr2/wanderer/decnet.PNG)
  
  * random variables: Pose, Point Cloud  <br>
  * utility: minimum distances derived from predicted robot state and predicted point cloud using all possible actions (linear and angular velocity)  <br>
  * decision: linear and angular velocity corresponding to the maximum expected utility

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
$ roslaunch wanderer test.launch
```

we can test the running range finder via

```bash
$ rostopic echo /scan
```

Now with the rangefinder node running and publishing to the topic, we can start the decision making node [`dec_node.py`](scripts/dec_node.py):

```bash
$ rosrun wanderer dec_node.py
```

Alternatively we created the launch file `run.launch`, which already starts the decision making node for the wanderer:

```bash
$ roslaunch wanderer run.launch
```

### Configuration

For the wanderer node we have two main configuration switches in the `dec_node.py`:
* `VIEW_ANGLE`: defines the cone of in front of the robot which describe the region of interest (ROI) for computing the utility. Only laser range measurements, where `abs(angle) < VIEW_ANGLE` is true are used. It is located at [line 13](https://github.com/tomas-thalmann/demos_ros/blob/089596d08b1fa8fe96442fde11d6d2d5779b42dc/wanderer/scripts/dec_node.py#L13) of `dec_node.py`.
* `MAX_THRESHOLD`: defines at which point a linear velocity of 0.0 is allowed. If the maximal measured distance within the ROI is less then `MAX_THRESHOLD` the robot is allowed to turn in place. It is located at [line 15](https://github.com/tomas-thalmann/demos_ros/blob/089596d08b1fa8fe96442fde11d6d2d5779b42dc/wanderer/scripts/dec_node.py#L15) of `dec_node.py`.

> Would be nice to pass these two parameters as arguments to be able to specify them in the launch file, but due to lack of time this feature was postponed... ;-)

