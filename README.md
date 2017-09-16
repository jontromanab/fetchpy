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
## Using the Head ##
The most basic option to move the head to a desired position is calling the MoveTo() function. This function can directly take joint values of the 'head_pan_joint' and 'head_tilt_joint'. The limts of head_pan_joint is [-1.57, 1.57] and head_tilt_joint is [-0.76, 1.45]. e.g. For looking at a position at the lower left, you can pass:
```
robot.head.MoveTo([-1.42, 1.34])
```
You can also move the robot head to presaved positions by:
```
robot.head.PlanToNamedConfiguration('look_up', execute=True)
```
to look up. Other pre-saved configurations are ``'look_straight','look_down','look_right','look_left'``. The ``execute = True`` argument is the most important argument, without this the system will only plan for the configuration and return an untimed trajectory, but will not execute it.
You can also move the head to look at a point in the environment specified by a position vector. This command is specifically useful if you want to track the endeffector for any task. To look at a point at 3.0 in x, -2.0 in y and 4.0 in z:
```
robot.head.LookAt([3.0, -2.0, 4.0])
```
If you need the current state of the joints in the head, you can call:
```
robot.head.GetJointState()
```
or if you just want to get the index of the head joints:
```
robot.head.GetIndices()
```
## Using the Gripper ##
The gripper of the Fetch robot can be easily controlled by the ``MoveHand()`` method and providing a value. 0.05 will open the gripper half-way
```
robot.gripper.MoveHand(0.05)
```
while 0.0 closes the gripper and 0.1 opens the gripper maximally
The Gripper can also be opened and closed by NamedConfigurations
```
robot.gripper.MoveToNamedConfiguration('closed',execute = True)
robot.gripper.MoveToNamedConfiguration('open',execute = True)
```
or by just commanding to open or close
```
robot.gripper.CloseHand()
robot.gripper.OpenHand()
```
To obtain the indices and names of the gripper joints 
```
robot.gripper.GetFingerIndices()
robot.gripper.GetJointNames()
```
## Using the Base ##
To Move the base, the velocity can be set as:
```
robot.base.Move([1.0, 0.5])
```
while the first component is translation in x-direction, and the second component is rotation is z-direction. Due the non-holonomic condition, the robot can only be moved in this two directions.
To move the base forward or backward
```
robot.base.Forward(-0.5)
```
or to rotate the base
```
robot.base.Rotate(0.5)
```

## Using the arm ##
The manipulator of the Fetch robot is 7DOF arm. To get the current DOF values of the arm. To get the joint names and current joint values of the arm:
```
robot.arm.GetJointNames()
robot.arm.GetDOFValues()
```
To plan the endeffector to reach a position and orientation in the environment, you can directly provide the 4x4 homogeneous transformation matrix in the world coordinate, assuming the robot base is at the origin.
```
import numpy
Tgoal = numpy.array([[0.49946526,0.8461969,0.18570209,0.45915831],
                    [-0.30122926,0.37060927,-0.87858392,0.3065221], 
                    [-0.8122779,0.38288324,0.44000572,0.9], 
                    [0,0,0,1]])
robot.arm.PlanToEndEffectorPose(Tgoal,execute = True)
```
You can directly call the IK Solution too.
```
filter_options = openravepy.IkFilterOptions.CheckEnvCollisions # or 0 for no collision checks
config = robot.arm.FindIKSolution(Tgoal, filter_options) # will return None if no config can be found
robot.arm.PlanToConfiguration(config, execute=True)
```

The robot will plan a path to the position and orientation defined by the transformation matrix from its current pose and execute it as the ``execute = True`` paramater is provided. Every plan returns a trajectory. If you don't want to execute the trajectory right now, you should omit this parameter. If you want to save the trajectory and execute it later:
```
from prpy.rave import save_trajectory
traj = robot.arm.PlanToEndEffectorPose(Tgoal)
save_trajectory(traj,'/home/buddy/Desktop/my_new_traj.xml')
```
You can load this saved trajectory later. Beware, this trajectory is untimed. To execute this trajectory, we can run executePath(traj) which will time and preprocess it internally. Also if you are planning to exeucte saved trajectory later, the starting state of the robot should be the same as the starting state of the trajectory.
```
from prpy.rave import load_trajectory
env = robot.GetEnvironment()
traj = load_trajectory(env, '/home/buddy/Desktop/my_new_traj.xml')
robot.ExecutePath(traj)
```
You can also plan for an offset from its current position. The following piece of code will mode the arm .1 meters in the +z direction (UP)
```
distance = .1
direction = [0,0,1]
robot.arm.PlanToEndEffectorOffset(direction, distance, execute=True)
```

You can also plan the arm to a configuration.
```
angle = ([0.35, -0.5,0.5,0.75,-0.55,0.78,0.9]) 
robot.arm.PlanToConfiguration(angle, execute = True) 
```
There are also simple named configurations that can be easily setup. You can plan for a straight arm or docked arm configuration by:
```
robot.arm.PlanToNamedConfiguration('arm_dock', execute=True)
robot.arm.PlanToNamedConfiguration('straight', execute=True)
```

## Using the arm and torso ##
The arm and torso combined also makes a 8DOF manipulator where 7DOF is associated with the arm and 1 extra DOF is from the prismatic joint of the torso. The system calculates a 8DOF IK solution for planning with arm and torso. All the valid commands for arm manipulation can also be executed on this manipulator too.  e.g. To plan and execute a trajectory defined by joint configuration:
```
angle = ([0.3, 0.35, -0.5,0.5,0.75,-0.55,0.78,0.9]) 
robot.arm_torso.PlanToConfiguration(angle, execute = True) 
```
Though there are no named configurations for arm and torso, it can be easily incorporated. 
**Beware: If you are controlling the robot in Gazebo, there is a bug in Fetch simulation controller(only). While planning for arm and torso and coming back to planning for arm, the torso suddenly goes down to 0.0 position. This bug is reported in** https://github.com/fetchrobotics/robot_controllers/issues/32 

## Using the Whole Body(arm, torso and Base) ##
Still in Progress. Coming Soon


## Demo ##
To view the capabilites of Fetchpy, the best starting point is pose_control.py. Run your roscore, start the real robot or robot in gazebo (roslaunch fetch_gazebo simulation.launch) and in a different terminal:
```
rosrun fetchpy pose_control.py
```
It will bring up this options window and you can perform different predefined actions on robot.
```
Press 0 to dock the arm.
Press 1 to dock the arm and the torso.
Press 2 to say something.
Press 3 to straight the arm.
Press 4 to go to cleaing wall.
Press 5 to wave.
Press 6 to do I love you.
Press 7 to say Yes.
Press 8 to say No.
Press 9 to quit. 
```
