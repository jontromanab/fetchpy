import numpy, openravepy, logging
from prpy.base import MobileBase
import prpy, time
from prpy.controllers import OrController
from openravepy import *

import rospy
from geometry_msgs.msg import (Twist, Vector3)

logger = logging.getLogger('fetchpy')

def or_traj_to_ros_vel(robot,traj):
	vel = []
	cspec = traj.GetConfigurationSpecification()
	for iwaypoint in xrange(traj.GetNumWaypoints()):
		waypoint = traj.GetWaypoint(iwaypoint)
		affdofvalues = cspec.ExtractAffineValues(waypoint, robot, 11, 1)
		vel_imd = []
		vel_imd.append(affdofvalues[0])
		vel_imd.append(affdofvalues[2])
		vel.append(vel_imd)
	return vel



class BaseVelocityPublisher(object):
	def __init__(self, ns, controller_name, timeout = 0.0):
		self.log = logging.getLogger(__name__)
		as_name = ns + '/' + controller_name+'/command'
		self._pub = rospy.Publisher(as_name,Twist, queue_size=1)

	def execute(self, vel):
		goal_msg = Twist()
		curr_vel = [0.0, 0.0]
		disc_vel = numpy.array(vel*10)
		for i in range(10):
			goal_msg.linear.x = disc_vel[0]
			goal_msg.angular.z = disc_vel[1]
			self._pub.publish(goal_msg)
			rospy.sleep(0.1)

class BaseVelocityController(OrController):
	def __init__(self, namespace, robot, controller_name, simulated = False, timeout = 10.0):
		if simulated:
			raise NotImplementedError('Simulation not supported here')
		self.logger = logging.getLogger(__name__)
		self.namespace = namespace
		self.controller_name = controller_name
		self.Publisher = BaseVelocityPublisher(namespace, controller_name, timeout)
		self._current_cmd = None
		self.robot = robot
		self.logger.info('Base velocity controller {}/{} initialized'
			.format(namespace,controller_name))

	def SetPath(self, traj):
		if not self.IsDone():
			self.logger.warning('Base controller is alreday in progress. You should wait')
		vel = or_traj_to_ros_vel(self.robot, traj)
		self.logger.info('Moving by: {}'.format(vel[1]))
		for i in vel:
			self._current_cmd = self.Publisher.execute(i)

	def IsDone(self):
		return self._current_cmd is None or self._current_cmd.done()


class BASE(MobileBase):
	def __init__(self, sim, robot):
		MobileBase.__init__(self, sim = sim, robot = robot)
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		self.robot = robot

		if not self.simulated:
			self.controller = BaseVelocityController('',self.robot, 'base_controller')
			
		else:
			self.controller = robot.AttachController(name = robot.GetName(),
				args='NavigationController {0:s} {1:s}'.format('fetchpy', 
					'/navcontroller'),dof_indices=[],affine_dofs=
				openravepy.DOFAffine.Transform,simulated=sim)

	def CloneBindings(self, parent):
		MobileBase.CloneBindings(self, parent)

	def Forward(self, meters, execute = True, timeout= None, **kwargs):
		return MobileBase.Forward(self, meters, execute=execute,
				timeout=timeout, **kwargs)
		

	def Rotate(self, angle_rad, execute = True, timeout = None, **kwargs):
		return MobileBase.Rotate(self, angle_rad, execute=execute,
				timeout=timeout, **kwargs)
		

	def Move(self, vel, execute = True, timeout = None, **kwargs):
		if self.simulated:
			self.Rotate(vel[1])
			self.Forward(vel[0])
			
		else:
			self.controller.SetDesired(vel)
			is_done = prpy.util.WaitForControllers([self.controller], timeout=
					timeout)

	def DriveAlongVector(self, direction, goal_pos):
		"""
		Rotate to face the given direction, then drive to goal position.
		@param direction a 2 or 3 element direction vector
		@param goal_pos a 2 or 3 element position vector(world frame)
		"""
		direction = numpy.array(direction[:2])/numpy.linalg.norm(direction[:2])
		robot_pose = self.robot.GetTransform()
		distance = numpy.dot(numpy.array(goal_pos[:2]) - robot_pose[:2, 3], direction)
		cur_angle = numpy.arctan2(robot_pose[1, 0], robot_pose[0, 0])
		des_angle = numpy.arctan2(direction[1], direction[0])
		self.Rotate(des_angle - cur_angle)
		self.Forward(distance)












		



