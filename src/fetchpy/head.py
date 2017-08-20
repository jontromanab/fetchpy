import numpy, openravepy
from prpy import util
from prpy.base.endeffector import EndEffector
from prpy.base.manipulator import Manipulator
from prpy.planning import PlanningError
from prpy.controllers import RewdOrTrajectoryController
from ros_control_client_py import SetPositionFuture
from arm import ARM


import logging
import rospy
import actionlib

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint, JointTrajectory)


class HEAD(ARM):
	def __init__(self, robot, sim, namespace):
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		self.robot = robot
		env = self.robot.GetEnv()

		#Controller Setup
		self.namespace = namespace
		if not sim:
			self.controller = RewdOrTrajectoryController(self,'',
				'head_controller', self.GetJointNames())
		else:
			self.controller = self.robot.AttachController(name=self.GetName(),
				args = 'IdealController', dof_indices = self.GetIndices(),
				affine_dofs = 0, simulated = sim)
		



	def GetJointNames(self):
		jointnames = ['head_pan_joint','head_tilt_joint']
		return jointnames

	def SetActive(self):
		self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])

	def GetIndices(self):
		#self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])
		self.SetActive()
		values =  self.robot.GetActiveDOFIndices()
		return values
	
	def GetJointState(self):
		#self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])
		self.SetActive()
		values = self.robot.GetActiveDOFValues()
		return values

	def GetName(self):
		return 'head'

	def GetMaxVelocity(self):
		self.SetActive()
		values = self.robot.GetActiveDOFMaxVel()
		return values


	def CreateTrajectory(value):
		max_vel = self.GetMaxVelocity()
		curr_val = self.GetJointState()
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()
		traj_msg.header.seq = 1
		traj_msg.jointnames = self.GetJointNames()
		



	def MoveTo(self, value, timeout = None):
		if self.simulated:
			self.controller.SetDesired(value)
			util.WaitForControllers([self.controller], timeout = timeout)







