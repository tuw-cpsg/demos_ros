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
  
  Publishing twist messages containing actions:
  ```
  # store the commands and publish to the topic
  self.ang_vel = candidate[1]
  self.lin_vel = candidate[0]
  twist = Twist()
  twist.linear.x = self.lin_vel
  twist.angular.z = self.ang_vel
  self.pub.publish(twist)
  ```
  
* evidence
  
  For decision-making the point-cloud provided by the Hoyuko Laserscanner acts as evidence.
  
  Code to provide Laserscanner data as array containing the ranges with corresponding angle:
  ```
    def callback_laser(self, data):
        ranges, angles = self._convert_data(data)
        self.laser_obs = np.array([angles, ranges]).T

    def _convert_data(self, data):
        ranges = np.array(data.ranges)
        angles = np.arange(len(data.ranges))
        angles = data.angle_min + angles * data.angle_increment
        idx = ~np.isnan(angles)
        angles = angles[idx]
        ranges = ranges[idx]
        idx2 = ~np.isnan(ranges)
        angles = angles[idx2]
        ranges = ranges[idx2]
        idx3 = ranges <= 100.
        angles = angles[idx3]
        ranges = ranges[idx3]
        return ranges, angles
  ```
  
* (next) state
  
  The actual state of the robot (x- and y-coordinate and theta) is assumed to be (0[m],0[m],0[rad]) for each iteration of the decision making process.
  The actual state is predicted by using the possible set of actions, which are chosen to be: 
  lin_vel = (0.05, 0.1)[m/s] and ang_vel = (-0.3, -0.25, ..., 0.25, 0.3)[rad/s]
  Thus, the prediction equations are:
  ```
  $ d_i = lin_vel_i * dt $
  $ theta_j = ang_vel_j * dt $
  $ x_ij = d_i * cos(theta_j) $
  $ y_ij = d_i * sin(theta_j) $
  ```
  Code to predict robot pose:
  ```
    def predict_rel_pose(self, v, w):
        """
        Predict the movement of the robot (corresponding to the kinematic model of the KF estimator)
        :param v: linear velocity [m/s]
        :param w: angular velocity [rad/s]
        :return: transformation parameter
        """
        tp = w * self.dt
        xp = v * self.dt * m.cos(tp)
        yp = v * self.dt * m.sin(tp)
        return [-xp, -yp], -tp
  ```
  
  Additionally, the point-cloud from the Laserscanner is predicted by using d_i as translation and theta_p_j as rotation in a transformation.
  
  Metthods for predicting/transforming pointcloud:
  ```
  def predict_scan_points(a, d, t, rot):
    """
    Transforms polar coordinates by using ´transform_polar()´ and returns again polar elements
    :param a: angles [rad]
    :param d: distances [m]
    :param t: translation list of length 2 [x, y] in [m]
    :param rot: rotation angle [rad]
    :return: transformed polar coordinates, (angles, distances) tuple
    """
    x, y = transform_polar(a, d, t, rot)
    a2, d2 = cart2polar(x, y)
    return a2, d2
	
  def transform_polar(a, d, t, rot):
    """
    Transforms polar coordinates by translation and rotation parameter
    :param a: angles [rad]
    :param d: distances [m]
    :param t: translation list of length 2 [x, y] in [m]
    :param rot: rotation angle [rad]
    :return: transformed coordinates (x, y) tuple
    """
    x2 = t[0] + np.cos(a + rot) * d
    y2 = t[1] + np.sin(a + rot) * d
    return x2, y2
	
  def cart2polar(x, y):
    """
    Converts cartesian coordinates (right handed) to polar coordinates
    :param x: coordinate [m]
    :param y: coordinate [m]
    :return: (angles, distances) tuple
    """
    d = np.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    return a, d
  ```
  
* utility
  
  The expected utility u_ij is chosen to be the minimum of a set of distances derived from the predicted robot position (x,y)_ij and the corresponding predicted point-cloud pc_ij.
  The set of distances is calculated between (x,y)_ij and the points from pc_ij which are within a certain field of view of the robot with a chosen aperture angle of 90°.
  The best actions lin_vel_opt and ang_vel_opt correspond to the maximum expected utility u_ij_max.
  
  Additionally, a check on the maximum distance in each set of distances is made to avoid that the robot get stuck (e.g. in room corners).
  If these maximum distances are all smaller than 3m, lin_vel = 0m/s is added to the possible actions.
  
  P(s'|a,e) is assumed to be equal for each computed next state and therefore can be unconsidered.  
  
  Callback- and utility method where utilities are computed:
  ```
    def callback_new(self, data):
        """
        Takes the latest point cloud, does the simple utility based decision making and publishes the result to the
        /teleop to wander around
        :param data: data of the odometry pose (unused)
        :return:
        """
        if self.laser_obs is None:
            # skip if no laser range data available
            return

        if self.max_dist < MAX_THRESHOLD:
            # if there is no region left to explore (maximal distance of laser range finder is smaller than a threshold)
            # use extended set of linear velocity (including 0.0)
            vs = self.range_v2
        else:
            # otherwise force the robot to keep moving
            vs = self.range_v

        # create all possible combinations of linear and angular velocity
        combinations = list(product(vs, self.range_w))

        utilities = None
        for c in combinations:
            # iterate over all combinations and compute the utility
            u = self.utility(c[0], c[1])
            if utilities is None:
                utilities = u
            else:
                utilities = np.vstack((utilities, u))
            
        # select the command set with the maximal expected utility (MEU)
        candidate = utilities[np.argmax(utilities[:, 2]), :]
		...
		
	def utility(self, lin_vel, ang_vel):
        """
        Takes a pair of commands and computes the expected utility
        :param lin_vel: linear velocity
        :param ang_vel: angular velocity
        :return: row vector containing the commands + utility + maximal distance
        """
        # predict the relative pose change introduced by the specified commands
        pos, t = self.predict_rel_pose(lin_vel, ang_vel)
        # predict the point cloud under the assumption that the environment does not change
        ak, dk = predict_scan_points(self.laser_obs[:, 0], self.laser_obs[:, 1], pos, t)
        # subselect the points within the front area
        df = select_front_distances(ak, dk)
        # utility is defined by the minimal distance (nearest obstacle)
        # the further away the nearest obstacle the better
        util = np.nanmin(df)
        # store furthers obstacle
        max_dist = np.nanmax(df)
        return np.array([[lin_vel, ang_vel, util, max_dist]])
  ```
  
* decision network
  
  ![alt text](https://github.com/tomas-thalmann/demos_ros/blob/wanderer-gr2/wanderer/decnet.PNG)
  
  random variables: Pose, Point Cloud
  utility: minimum distances derived from predicted robot state and predicted point cloud using all possible actions (linear and angular velocity)
  decision: linear and angular velocity corresponding to the maximum expected utility

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

