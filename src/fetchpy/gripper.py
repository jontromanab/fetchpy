import numpy, openravepy
from prpy import util
from prpy.base.endeffector import EndEffector
from prpy.planning import PlanningError
from prpy.controllers import OrController
from ros_control_client_py import SetPositionFuture


import logging
import rospy
import actionlib
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)



class GripperCommandClient(object):
	def __init__(self, ns, controller_name, timeout = 0.0):
		"""Constructs a client that executes GripperCommandAction messages
		@param ns: namespace for gripper
		@type ns: str
		@param controller_name: name of the controller
		@type controller_name: str
		"""
		self.log = logging.getLogger(__name__)
		as_name = ns + '/' + controller_name+'/gripper_action'
		self._client = actionlib.SimpleActionClient(as_name, GripperCommandAction,)
		if not self._client.wait_for_server(rospy.Duration(timeout)):
			raise Exception('Could not connect to the action server {}'.format(as_name))

	def execute(self, position):
		"""Execute a given position
		@param position: positiongoal
		@type position: float
		"""
		goal_msg = GripperCommandGoal()
		goal_msg.command.position = position
		goal_msg.command.max_effort = 0.0
		
		#self.log.info('Sending positionGoal: {}'.format(goal_msg))
		#action_future = SetPositionFuture(position)
		#action_future._handle = 
		self._client.send_goal(goal_msg)
			#transition_cb=action_future.on_transition,
			#feedback_cb=action_future.on_feedback)
		#return action_future


class GripperCommandController(OrController):
	"""A controller for sending grippercommand action"""
	def __init__(self,namespace, controller_name, simulated = False, timeout = 10.0):
		if simulated:
			raise NotImplementedError('Simulation not supported here')
		self.logger = logging.getLogger(__name__)
		self.namespace = namespace
		self.controller_name = controller_name
		self.controller_client = GripperCommandClient(namespace, controller_name, timeout)
		self._current_cmd = None
		self.logger.info('Gripper Command controller {}/{} initialized'
			.format(namespace, controller_name))

	def SetDesired(self, position):
		if not self.IsDone():
			self.logger.warning('Gripper controller is alreday in progress. You should wait')
		self.logger.info('Sending position: {}'.format(position))
		self._current_cmd = self.controller_client.execute(position)

	def IsDone(self):
		return self._current_cmd is None or self._current_cmd.done()


class GRIPPER(EndEffector):
	def __init__(self, sim, manipulator, namespace):
		"""End effector wrapper for fetch gripper
		@param sim: whether the hand is simulated
		@manipulator: manipulator the gripper is attached to arm/arm_torso
		"""
		EndEffector.__init__(self,manipulator)
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		

		#Setting closing direction. Hope we dont need it as the gripper is parallel
		gripper_indices = manipulator.GetGripperIndices()
		closing_direction = numpy.zeros(len(gripper_indices))
		finger_indices = self.GetFingerIndices()

		for i, dof_index in enumerate(gripper_indices):
			if dof_index in finger_indices:
				closing_direction[i] = -1.

		manipulator.SetChuckingDirection(closing_direction)

		self.robot = self.manipulator.GetRobot()
		env = self.robot.GetEnv()

		#Controller Setup
		self.namespace = namespace
		if not sim:
			self.controller = GripperCommandController('','gripper_controller')
		else:
			self.controller = self.robot.AttachController(name = self.GetName(),
			args = 'IdealController', dof_indices = self.GetIndices(), affine_dofs = 0,
			simulated = sim )



	def MoveHand(self, value, timeout = None):
		"""Change the gripper joint with the value
		@param value: desired value of the joint
		"""
		if value>0.1 or value<0.0:
			self.logger.warning('The range is 0.0 and 0.1. try to provide value between this range')
		if self.simulated:
			self.controller.SetDesired([value, value])
			util.WaitForControllers([self.controller], timeout=timeout)
		else:
			self.controller.SetDesired(value)
			util.WaitForControllers([self.controller], timeout=timeout)

	def MoveToNamedConfiguration(self, name, timeout = None):
		"""Accesses the table provided in gripper_preshapes.yaml file
		"""
		try:
			configurations = self.robot.configurations
		except AttributeError:
			raise PlanningError('{:s} does not have a table of named'
				' configurations.'.format(robot))

		try:
			dof_indices, dof_values = self.robot.configurations.get_configuration(name)
		except KeyError:
			raise PlanningError('{0:s} does not have named configuration "{1:s}".'.format(self.robot, name))

		if not dof_values[0] == dof_values[1]:
			raise PlanningError('Both the values should be same')

		self.MoveHand(value = dof_values[0])


	def OpenHand(self,timeout = None):
		"""Opens the gripper by Release Fingers in task manipulation
		"""
		if self.simulated:
			robot = self.manipulator.GetRobot()
			p = openravepy.KinBody.SaveParameters
			with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
				self.manipulator.SetActive()
				robot.task_manipulation.ReleaseFingers()
			util.WaitForControllers([self.controller], timeout = timeout)
		else:
			self.MoveHand(value = 0.1)

	def CloseHand(self,timeout = None):
		"""Closes the gripper by Close Fingers in task manipulation
		"""
		if self.simulated:
			robot = self.manipulator.GetRobot()
			p = openravepy.KinBody.SaveParameters
			with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
				self.manipulator.SetActive()
				robot.task_manipulation.CloseFingers()
			util.WaitForControllers([self.controller], timeout = timeout)
		else:
			self.MoveHand(value = 0.0)
		

	def GetJointState(self):
		jointnames=(['l_gripper_finger_joint','r_gripper_finger_joint'])
		self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in jointnames])
		values = self.robot.GetActiveDOFValues()
		return values


	def GetJointNames(self):
		jointnames=(['l_gripper_finger_joint','r_gripper_finger_joint'])
		return jointnames

	def GetFingerIndices(self):
		"""Gets the DOF indices of the fingers.
		These are returned in the order: [l_joint, r_joint]
		@return DOF indices of the fingers
		"""
		return self.manipulator.GetGripperIndices()
		

	def GetIndices(self):
		return self.GetFingerIndices()

	def _GetJointFromName(self, name):
		robot = self.manipulator.GetRobot()
		print self.manipulator.GetName()
		full_name = '/{:s}/{:s}'.format(self.manipulator.GetName(), name)
		return robot.GetJoint(full_name)


		





