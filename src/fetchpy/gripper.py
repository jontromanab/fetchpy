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

		





