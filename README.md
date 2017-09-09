fetchpy
======
Fetchpy is a library for manipulating Fetch robot by OpenRAVE. 
Fetch is a mobile manipulator from [Fetch Robotics] (http://fetchrobotics.com/) with a 7DOF arm, a torso, a pan-tilt head and non-holonomic base. Fetchpy is highly inspired from [HerbPy] (https://github.com/personalrobotics/herbpy) developed in [Personal Robotics Lab] (https://personalrobotics.ri.cmu.edu)

fetchpy extensively uses the [PrPy] (https://github.com/personalrobotics/prpy) and [ros_control_client] (https://github.com/personalrobotics/ros_control_client) library functionalities.


## Installation ##

clone this repository in your workspace.
```
$ git clone https://github.com/jontromanab/fetchpy
$ cd ..
$ catkin_make
```

## Running Fetchpy ##
Calling the ``initialize`` function in your script, you can call all the fetchpy related components in openrave
```
env, robot = fetchpy.initialize()
```
By default, this function loads the OpenRAVE plugins necessary to communicate with Fetch's hardware drivers. You can run fetchpy in simulation mode by passing the option ``sim=True``. With default settings an RViz viewer will be attached. You can set the viewer to qtcoin to passing the ``--viewer qtcoin`` argument. 

## Fetchpy Console ##
There are 3 modes supported on fetchpy. The ``roscore`` should be running in all the modes:

### 1. Running only in OpenRAVE: ###
In this mode, you can set the robot to openrave simulation mode and can control the robot only in openrave. 
```
$ rosrun fetchpy console.py --sim
```
### 2. Running the robot in Gazebo: ###
In this mode, you can control the gazebo simulated robot through fetchpy. The gazebo package for fetch robot can be accessed from 
[fetch_gazebo] (https://github.com/fetchrobotics/fetch_gazebo)
```
$ roslaunch fetch_gazebo simulation.launch
$ rosun fetchpy console.py
```
### 3. Running the real robot: ###
This mode, controls the real robot. Make sure the robot is at a safe distance from human and the robot workspace is not in probable collision with the environment. Access the robot and run:
```
$ rosrun fetchpy console.py 
```
## Accessing Individual Components ##
The robot returned by fetchpy.initialize() is an OpenRAVE robot. This object provides access to individual components of fetch robot.

* ``robot.head`` : The panning and tilting head of the robot
* ``robot.gripper`` : The parallel gripper of the robot
* ``robot.arm`` : The 7DOF arm of the robot
* ``robot.arm_torso`` : The 7DOF arm and the torso
* ``robot.base`` : The non-holonomic base of the robot
## Using the head ##
