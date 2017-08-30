import numpy, openravepy, logging
import prpy, time, rospy
from openravepy import *
import actionlib

from .head import FollowJointTrajectoryController
from .base import BaseVelocityController

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint, JointTrajectory)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (Twist, Vector3)

logger = logging.getLogger('fetchpy')

# class WholeBodyPublisher(object):
# 	def __init__(self,ns, robot, arm_controller_name, sim = False,timeout = 0.0):
# 		self.log = logging.getLogger(__name__)
# 		self.namespace = ns
# 		self.robot = robot
# 		self.base_controller = BaseVelocityController('','base_controller')
# 		self.manip_controller = FollowJointTrajectoryController(self,'',
# 				arm_controller_name, self.GetJointNames())

# 	def execute(self, values):
# 		base_vel = values[-2:]
# 		manip_traj = self.createArmTrajectory(values[:-2])
# 		self.base_controller.SetDesired(base_vel)
# 		self.manip_controller.SetPath(manip_traj)

	# def createArmTrajectory(self, values):
	# 	traj_msg = JointTrajectory()
	# 	traj_msg.header.stamp = rospy.Time.now()+rospy.Duration.from_sec(0.0)
	# 	traj_msg.joint_names = self.GetJointNames()
	# 	traj_msg.points = []

	# 	orig_point = JointTrajectoryPoint()
	# 	orig_point.positions = self.getJointState()
	# 	orig_point.velocities = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	orig_point.accelerations = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	orig_point.time_from_start = rospy.Duration.from_sec(0.0)
	# 	traj_msg.points.append(orig_point)

	# 	end_point = JointTrajectoryPoint()
	# 	end_point.positions = values
	# 	end_point.velocities = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	end_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# 	end_point.time_from_start = rospy.Duration.from_sec(1.0)
	# 	traj_msg.points.append(end_point)

	# 	# goal_msg = FollowJointTrajectoryGoal()
	# 	# goal_msg.trajectory = traj_msg
	# 	return traj_msg

	# def getJointState(self):
	# 	self.SetActive()
	# 	values =  self.robot.GetActiveDOFIndices()
	# 	return values

	# def SetActive(self):
	# 	self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])

	# def GetJointNames(self):
	# 	jointnames = ['shoulder_pan_joint',
	# 	'shoulder_lift_joint',
	# 	'upperarm_roll_joint',
	# 	'elbow_flex_joint',
	# 	'forearm_roll_joint',
	# 	'wrist_flex_joint',
	# 	'wrist_roll_joint']
	# 	return jointnames




# class WholeBodyController():
# 	def __init__(self, namespace,robot,simulated = False, timeout = 10.0):
# 		self.logger = logging.getLogger(__name__)
# 		self.robot = robot
# 		self.sim = simulated
# 		self.namespace = namespace
# 		self._current_cmd = None
# 		self.Publisher = WholeBodyPublisher(namespace, robot,'arm_controller', sim=self.sim)


# 	def SetDesired(self, joint_values):
# 		if not self.IsDone():
# 			self.logger.warning('Whole body controller is already in progress. Please wait')
# 		self.Publisher.execute(joint_values)

# 	def IsDone(self):
# 		return self._current_cmd is None or self._current_cmd.done()


# class WholeBody():
# 	def __init__(self, sim, robot):
# 		self.simulated = sim
# 		self.logger = logging.getLogger(__name__)
# 		self.robot = robot
# 		self.controller = WholeBodyController('',self.robot,sim,'whole_body_controller')

# 	def Move(self, joint_values):
# 		self.controller.SetDesired(joint_values)

class WholeBodyController(object):
	def __init__(self, ns, robot, sim = False, timeout = 0.0):
		self.log = logging.getLogger(__name__)
		self.robot = robot
		self.client_ = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction,)
		self.pub_ = rospy.Publisher('/base_controller/command', Twist, queue_size=1)

	def execute(self, values):
		#print 'I am executing: '+str(values)
		goal_msg_base = Twist()
		vel = values[-2:]
		goal_msg_base.linear.x = vel[0]
		goal_msg_base.angular.z = vel[1]
		#print goal_msg_base
		arm_traj = self.createArmTrajectory(values[:-2])
		goal_msg = FollowJointTrajectoryGoal()
		goal_msg.trajectory = arm_traj
		self.client_.send_goal(goal_msg)
		#self.client_.wait_for_result()
		self.pub_.publish(goal_msg_base)


	def createArmTrajectory(self, values):
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration.from_sec(0.0)
		traj_msg.joint_names = self.GetJointNames()
		traj_msg.points = []

		# orig_point = JointTrajectoryPoint()
		# orig_point.positions = self.getJointState()
		# orig_point.velocities = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		# orig_point.accelerations = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0]
		# orig_point.time_from_start = rospy.Duration.from_sec(0.0)
		# traj_msg.points.append(orig_point)

		end_point = JointTrajectoryPoint()
		end_point.positions = values
		end_point.velocities = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		end_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		end_point.time_from_start = rospy.Duration.from_sec(1.0)
		traj_msg.points.append(end_point)

		# goal_msg = FollowJointTrajectoryGoal()
		# goal_msg.trajectory = traj_msg
		return traj_msg

	def getJointState(self):
		self.SetActive()
		values =  self.robot.GetActiveDOFIndices()
		return values

	def SetActive(self):
		self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])

	def GetJointNames(self):
		jointnames = ['shoulder_pan_joint',
		'shoulder_lift_joint',
		'upperarm_roll_joint',
		'elbow_flex_joint',
		'forearm_roll_joint',
		'wrist_flex_joint',
		'wrist_roll_joint']
		return jointnames

class WholeBody():
	def __init__(self, sim, robot):
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		self.robot = robot
		self.controller = WholeBodyController('',self.robot,sim)

	def Move(self, joint_values):
		self.controller.execute(joint_values)






