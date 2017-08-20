import numpym openravepy
from prpy import utils
from prpy.base.endeffector import EndEffector
from prpy.controller import OrController
from ros_control_client_py import SetPositionFuture


import logging
import rospy



class GripperCommandClient(object):
	def __init__(self, ns, controller_name, timeout = 0.0):
		"""Constructs a client that executes GripperCommandAction messages
		@param ns: namespace for gripper
		@type ns: str
		@param controller_name: name of the controller
		@type controller_name: str
		"""
		import rospy
		import actionlib
		from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)

		self.log = logging.getLogger(__name__)
		as_name = ns + '/' + controller_name+'/gripper_action'
		self.client = actionlib.SimpleActionClient(as_name, GripperCommandAction,)
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
		
		self.log.info('Sending positionGoal: {}'.format(goal_msg))
		action_future = SetPositionFuture(position)
		action_future._handle = self._client.send_goal(goal_msg,
			transition_cb=action_future.on_transition,
			feedback_cb=action_future.on_feedback)
		return action_future


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


class Gripper(EndEffector):
	def __init__(self, sim, manipulator, namespace):
		"""End effector wrapper for fetch gripper
		@param sim: whether the hand is simulated
		@manipulator: manipulator the gripper is attached to arm/arm_torso
		"""
		EndEffector.__init__(self,manipulator)
		self.simulated = sim

		#Setting closing direction. Hope we dont need it as the gripper is parallel
		gripper_indices = manipulator.GetGripperIndices()
		closing_direction = numpy.zeros(len(gripper_indices))
		finger_indices = self.GetFingerIndices()

		for i, dof_index in enumerate(gripper_indices):
			if dof_index in finger_indices:
				closing_direction[i] = 1.

		manipulator.SetChuckingDirection(closing_direction)

		robot = self.manipulator.GetRobot()
		env = robot.GetEnv()

		#Controller Setup
		self.namespace = namespace
		if not sim:
			self.controller = GripperCommandController('gripper_controller')
		else:
			self.controller = robot.AttachController(name = self.GetName(),
			args = 'IdealController', dof_indices = self.getIndices(), affine_dofs = 0,
			simulated = sim )

	def MoveHand(self, value = None, timeout = None):
		"""Change the gripper joint with the value
		@param value: desired value of the joint
		"""
		curr_pos = self.GetDOFValues()
		self.controller.SetDesired(value)
		util.WaitForControllers([self.controller], timeout=timeout)

	def OpenHand(self,timeout = None):
		self.controller.SetDesired(0.0)
		util.WaitForControllers([self.controller], timeout=timeout)

	def CloseHand(self,timeout = None):
		self.controller.SetDesired(0.7)
		util.WaitForControllers([self.controller], timeout=timeout)

	def GetJointState(self):
		self.robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames])
		values = self.robot.GetActiveDOFValues()
		return values[0]


	def GetJointNames(self):
		jointnames=['l_gripper_finger_joint','r_gripper_finger_joint']
		return jointnames

	def getFingerIndices(self):
		"""Gets the DOF indices of the fingers.
		These are returned in the order: [l_joint, r_joint]
		@return DOF indices of the fingers
		"""
		return [self._GetJointFromName(name).GetDOFIndex() for name in self.GetJointNames]

	def getIndices(self):
		return self.getFingerIndices()

	def _GetJointFromName(self, name):
		robot = self.manipulator.GetRobot()
		full_name = '/{:s}/{:s}'.format(self.manipulator.GetName(), name)
		return robot.GetJoint(full_name)


		





