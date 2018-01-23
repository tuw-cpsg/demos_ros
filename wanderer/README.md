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
  [launch file format](http://wiki.ros.org/roslaunch/XML)). It is then enough
  to show, how to start the launch file -- optional (10 bonus points)
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

Finally, this repo shall include:
* A `src` folder containing the implementation.
* This README with answered questions and copy-and-paste instructions for how
  to start the demo (ROS nodes, topic redirects if necessary). In the best
  case, only a ROS launch file has to be executed. However, a list of commands
  is also ok.
* Optional launch file (please put into a folder called `launch`).

### Grading

| Points |                     |
|-------:|---------------------|
|     40 | problem formulation |
|     40 | implementation      |
|     20 | docs in this README |
|    +10 | launch file         |
