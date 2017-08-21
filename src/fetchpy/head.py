import numpy, openravepy
import time
from prpy import util
from prpy.base.endeffector import EndEffector
from prpy.base.manipulator import Manipulator
from prpy.planning import PlanningError
from prpy.controllers import (RewdOrController, OrController,)
from ros_control_client_py import SetPositionFuture
from arm import ARM

import logging
import rospy
import actionlib

from control_msgs.msg import (FollowJointTrajectoryAction, 
	FollowJointTrajectoryGoal,
	PointHeadAction, 
	PointHeadGoal)
from trajectory_msgs.msg import (JointTrajectoryPoint, JointTrajectory)
from geometry_msgs.msg import (PointStamped, Vector3)

class PointHeadClient(object):
	def __init__(self, ns, controller_name, timeout = 0.0):
		self.log = logging.getLogger(__name__)
		as_name = ns + '/' + controller_name + '/point_head'
		self._client = actionlib.SimpleActionClient(as_name,PointHeadAction,)
		if not self._client.wait_for_server(rospy.Duration(timeout)):
			raise Exception('Could not connect to the action server {}'
				.format(as_name))

	def execute(self, point_value):
		goal_msg = PointHeadGoal()
		point_st_msg = PointStamped()
		point_st_msg.header.frame_id = "/base_link"
		point_st_msg.header.stamp = rospy.Time.now()
		point_st_msg.point.x = point_value[0]
		point_st_msg.point.y = point_value[1]
		point_st_msg.point.z = point_value[2]
		goal_msg.target = point_st_msg
		self._client.send_goal(goal_msg)


class PointHeadController(OrController):
	def __init__(self,namespace, controller_name, simulated = False, timeout = 10.0):
		if simulated:
			raise NotImplementedError('Simulation not supported here')
		self.logger = logging.getLogger(__name__)
		self.namespace = namespace
		self.controller_name = controller_name
		self.controller_client = PointHeadClient(namespace, controller_name, timeout)
		self._current_cmd = None
		self.logger.info('Point Head controller {}/{} initialized'
			.format(namespace, controller_name))

	def SetDesired(self, point):
		if not self.IsDone():
			self.logger.warning('Point Head Controller is already in progress. You should wait')
		self.logger.info('Looking at: {}'.format(point))
		self._current_cmd = self.controller_client.execute(point)

	def IsDone(self):
		return self._current_cmd is None or self._current_cmd.done()


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
			self.look_at_controller = PointHeadController('','head_controller')
		else:
			self.controller = self.robot.AttachController(name=self.GetName(),
				args = 'IdealController', dof_indices = self.GetIndices(),
				affine_dofs = 0, simulated = sim)

	def LookAt(self, value, timeout = None):
		self.look_at_controller.SetDesired(value)
		util.WaitForControllers([self.look_at_controller], timeout=timeout)


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

	def MoveToNamedConfiguration(self, name, timeout=None):
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

			









