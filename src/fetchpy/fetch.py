import logging
import os
import prpy
import prpy.dependency_manager
from prpy.collision import (BakedRobotCollisionCheckerFactory,SimpleRobotCollisionCheckerFactory,)

from openravepy import (
	Environment,
	RaveCreateModule,
	RaveCreateCollisionChecker,
	RaveInitialize,
	openrave_exception,
)

#fetch base and FETCHROBOT
from .fetchrobot import FETCHRobot

logger = logging.getLogger('fetchpy')

def initialize(robot_xml = None, env_path = None, viewer = 'rviz',sim = False, **kw_args):
	prpy.logger.initialize_logging()


	#Hide TrajOpt logging.
	os.environ.setdefault('TRAJOPT_LOG_THRESH','WARN')

	#Load plugins
	prpy.dependency_manager.export()
	RaveInitialize(True)

	#Create the environment
	env = Environment()
	if env_path is not None:
		if not env.Load(env_path):
			raise ValueError(
				'Unable to load environment from path {:s}'.format(env_path))

	#Load the URDF file into OpenRave.
	urdf_module = RaveCreateModule(env, 'urdf')
	if urdf_module is None:
		logger.error('Unable to load or_urdf module. Do you have or_urdf'
			'built and installed in one of your Catkin workspaces?')
		raise ValueError('Unable to load or_urdf plugin.')

	urdf_uri = 'package://fetchpy/robot_description/fetch.gazebo.urdf'
	srdf_uri = 'package://fetchpy/robot_description/fetch.srdf'
	args = 'Load {:s} {:s}'.format(urdf_uri, srdf_uri)
	fetch_name = urdf_module.SendCommand(args)
	if fetch_name is None:
		raise ValueError('Failed loading FETCH model using or_urdf.')

	robot = env.GetRobot(fetch_name)
	if robot is None:
		raise ValueError('Unable to find robot with name "{:s}".'.format(
                         fetch_name))

	#Default to FCL.
	collision_checker = RaveCreateCollisionChecker(env, 'fcl')
	if collision_checker is not None:
		env.SetCollisionChecker(collision_checker)
	else:
		collision_checker = env.GetCollisionChecker()
		logger.warning('Failed creating "fcl", defaulting to the default OpenRAVE'
			' collision checker. Did you install or_fcl?')

	#Enable Baking if it is supported
	try:
		result = collision_checker.SendCommand('BakeGetType')
		is_baking_suported = (result is not None)
	except openrave_exception:
		is_baking_suported = False

	if is_baking_suported:
		robot_checker_factory = BakedRobotCollisionCheckerFactory()
	else:
		robot_checker_factory = SimpleRobotCollisionCheckerFactory()
		logger.warning('Collision checker does not support baking. Defaulting to'
			' the slower SimpleRobotCollisionCheckerFactory.')

	#Default arguments
	keys = ['arm_sim','arm_torso_sim','gripper_sim',
	'head_sim','torso_sim','base_sim','talker_sim','perception_sim']

	for key in keys:
		if key not in kw_args:
			kw_args[key] = sim

	prpy.bind_subclass(robot, FETCHRobot, robot_checker_factory = robot_checker_factory,
		**kw_args)

	if sim:
		dof_indices, dof_values = robot.configurations.get_configuration('straight')
		robot.SetDOFValues(dof_values, dof_indices)

	#Start by attempting to load or_rviz.
	viewers = ['qtcoin','rviz']
	if viewer is None:
		viewer = 'rviz'
	if viewer not in viewers:
		raise Exception('Failed creating viewer of type "{0:s}".'.format(viewer))

	if viewer == 'rviz':
		env.SetViewer(viewer)
		if env.GetViewer() is None:
			logger.warning('Loading the RViz viewer failed. Do you have or_interactive'
			' marker installed? Falling back on qtcoin.')
		viewer == 'qtcoin'

	if viewer == 'qtcoin':
		env.SetViewer(viewer)


	if viewer and env.GetViewer() is None:
		env.SetViewer(viewer)
		if env.GetViewer() is None:
			raise Exception('Failed creating viewer of type "{0:s}".'.format(viewer))

	# if attach_viewer == 'True':
	# 	attach_viewer = 'qtcoin'
	# 	env.SetViewer(attach_viewer)

	# 	# Fall back on qtcoin if loading or_rviz failed
	# 	if env.GetViewer() is None:
	# 		logger.warning('Loading the RViz viewer failed. Do you have or_interactive'
	# 			' marker installed? Falling back on qtcoin.')
	# 		attach_viewer = 'rviz'
	# if attach_viewer and env.GetViewer() is None:
	# 	env.SetViewer(attach_viewer)
	# 	if env.GetViewer() is None:
	# 		raise Exception('Failed creating viewer of type "{0:s}".'.format(attach_viewer))

	#Remove the ROS logging handler again. It might have been added when we
	# loaded or_rviz.
	prpy.logger.remove_ros_logger()
	return env, robot






