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

def create_affine_trajectory(robot, poses):
    doft = openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.RotationAxis
    cspec = openravepy.RaveGetAffineConfigurationSpecification(doft, robot)
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
    traj.Init(cspec)

    for iwaypoint, pose in enumerate(poses):
        waypoint = openravepy.RaveGetAffineDOFValuesFromTransform(pose, doft)
	traj.Insert(iwaypoint, waypoint)

    return traj

class BaseVelocityPublisher(object):
	def __init__(self, ns, controller_name, timeout = 0.0):
		self.log = logging.getLogger(__name__)
		as_name = ns + '/' + controller_name+'/command'
		self._pub = rospy.Publisher(as_name,Twist, queue_size=1)

	def execute(self, vel):
		goal_msg = Twist()
		curr_vel = [0.0, 0.0]
		disc_vel = numpy.array(vel)
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
		print 'There are: '+str(len(vel))+' velocity points'
		vel0 = vel[0]
		for i in vel:
			final_base_goal = [a - b for a, b in zip(i, vel[0])]
			self.logger.info('Moving by: {}'.format(final_base_goal))
			self._current_cmd = self.Publisher.execute(final_base_goal)
			vel0 = final_base_goal

	def SetDesired(self,vel):
		if not self.IsDone():
			self.logger.warning('Base controller is alreday in progress. You should wait')
		self.logger.info('Moving by: {}'.format(vel))
		self._current_cmd = self.Publisher.execute(vel)


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
		direction = numpy.array([ 1., 0., 0. ])
		with self.robot.GetEnv():
			start_pose = self.robot.GetTransform()
			offset_pose = numpy.eye(4)
			offset_pose[0:3, 3] = vel[0] * direction
			transl_pose = numpy.dot(start_pose, offset_pose)
			relative_pose = openravepy.matrixFromAxisAngle([vel[0], 0.,vel[1]])
			goal_pose = numpy.dot(transl_pose, relative_pose)
		traj = create_affine_trajectory(self.robot, [ start_pose, goal_pose ])
		self.robot.ExecutePath(traj, **kwargs)


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

