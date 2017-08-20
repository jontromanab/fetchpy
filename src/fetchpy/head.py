import numpy, openravepy
import time
from prpy import util
from prpy.base.endeffector import EndEffector
from prpy.base.manipulator import Manipulator
from prpy.planning import PlanningError
from prpy.controllers import RewdOrController
from ros_control_client_py import SetPositionFuture
from arm import ARM

import logging
import rospy
import actionlib

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint, JointTrajectory)

class FollowJointTrajectoryController(RewdOrController):
	def __init__(self, robot, namespace, controller_name, joint_names, simulated = False):
		super(FollowJointTrajectoryController, self).__init__(robot,
			namespace, joint_names, simulated)
		if simulated:
			raise NotImplementedError('Simulation not supported in '
				'FollowJointTrajectoryController')
		from ros_control_client_py import FollowJointTrajectoryClient

		self.controller_client = FollowJointTrajectoryClient(namespace, controller_name)
		self.current_trajectory = None
		self.logger.info('Follow joint Trajectory controller initialized')

	def SetPath(self, traj):
		from ros_control_client_py import TrajectoryExecutionFailed

		if not self.IsDone():
			raise TrajectoryExecutionFailed('Currently executing another '
				'trajectory', traj, None)

		self.current_trajectory = self.controller_client.execute(traj)

	def IsDone(self):
		return (self.current_trajectory is None or
			self.current_trajectory.done())

	
class HEAD(ARM):
	def __init__(self, robot, sim, namespace):
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		self.robot = robot
		env = self.robot.GetEnv()

		#Controller Setup
		self.namespace = namespace
		if not sim:
			self.controller = FollowJointTrajectoryController(self,'',
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
		self.SetActive()
		values =  self.robot.GetActiveDOFIndices()
		return values
	
	def GetJointState(self):
		self.SetActive()
		values = self.robot.GetActiveDOFValues()
		return values

	def GetName(self):
		return 'head'

	def GetMaxVelocity(self):
		self.SetActive()
		values = self.robot.GetActiveDOFMaxVel()
		return values

	def CreateTrajectory(self,value):
		max_vel = self.GetMaxVelocity()
		curr_val = self.GetJointState()
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()
		traj_msg.header.seq = 1
		traj_msg.joint_names = self.GetJointNames()
		traj_msg.points = []

		orig_point = JointTrajectoryPoint()
		orig_point.positions = self.GetJointState()
		orig_point.velocities = [0.0,0.0]
		orig_point.accelerations = [0.0,0.0]
		orig_point.time_from_start = rospy.Duration.from_sec(0.0)
		traj_msg.points.append(orig_point)

		end_point = JointTrajectoryPoint()
		end_point.positions = value
		end_point.velocities = [0.0,0.0]
		end_point.accelerations = [0.0,0.0]

		pan_transit = numpy.absolute((orig_point.positions[0]- value[0])/max_vel[0])
		tilt_transit = numpy.absolute((orig_point.positions[1]- value[1])/max_vel[1])
		end_point.time_from_start = rospy.Duration.from_sec(numpy.amax([pan_transit,tilt_transit])/15.)
		traj_msg.points.append(end_point)
		return traj_msg

	def MoveTo(self, value, timeout = None):
		if self.simulated:
			self.controller.SetDesired(value)
			util.WaitForControllers([self.controller], timeout = timeout)
		else:
			traj = self.CreateTrajectory(value)
			self.controller.SetPath(traj)
			util.WaitForControllers([self.controller], timeout = timeout)

	def MoveToNamedConfigurations(self, name, timeout=None):
		try:
			configurations = self.robot.configurations
		except AttributeError:
			raise PlanningError('{:s} does not have a table of named'
				' configurations.'.format(robot))
		try:
			dof_indices, dof_values = self.robot.configurations.get_configuration(name)
		except KeyError:
			raise PlanningError('{0:s} does not have named configuration "{1:s}".'
				.format(self.robot, name))
		self.MoveTo(dof_values)

			









