import logging
import os
import prpy
import prpy.dependency_manager
from prpy.collision import(BakedRobotCollisionCheckerFactory,
    SimpleRobotCollisionCheckerFactory,)

from openravepy import(Environment, RaveCreateModule, RaveCreateCollisionChecker, RaveInitialize, openrave_exception,)

#fetch parts import
from fetchrobot import FETCHRobot

logger = logging.getLogger('fetchpy')

def initialize(robot_xml = None, env_path = None, attach_viewer = True, sim = True, **kw_args):
	prpy.logger.initialize_logging()

	# Hide TrajOpt logging.
    #os.environ.setdefault('TRAJOPT_LOG_THRESH', 'WARN')


	#load plugin
	prpy.dependency_manager.export()
	RaveInitialize(True)

	#create the environment
	env = Environment()
	if env_path is not None:
		if not env.Load(env_path):
			raise ValueError('Unable to load Environment from path {:s}'.format(env_path))


	#load the URDF file into OPENRAVE
	urdf_module = RaveCreateModule(env, 'urdf')
	if urdf_module is None:
		logger.error('Unable to load or_urdf module. Do you have or_urdf'
			' built and installed in one of your Catkin workspaces?')
		raise ValueError('Unable to load or_urdf plugin.')

	urdf_uri = 'package://fetchpy/robots/fetch.gazebo.urdf'
	srdf_uri = 'package://fetchpy/robots/fetch.srdf'
	args = 'Load {:s} {:s}'.format(urdf_uri, srdf_uri)
	fetch_name = urdf_module.SendCommand(args)
	if fetch_name is None:
		raise ValueError('Failed loading Fetch model using or_urdf.')

	robot = env.GetRobot(fetch_name)
	if robot is None:
		raise ValueError('Unable to find robot with name "{:s}".'.format(fetch_name))


	###Collision Checker
	collision_checker = RaveCreateCollisionChecker(env, 'fcl')
	if collision_checker is not None:
		env.SetCollisionChecker(collision_checker)
	else:
		collision_checker = env.GetCollisionChecker()
		logger.warning('Failed creating "fcl", defaulting to the default OpenRAVE'
			' collision checker. Did you install or_fcl?')
	prpy.bind_subclass(robot, FETCHRobot, 
		robot_checker_factory = robot_checker_factory, **kw_args)
	if sim = True:
     	dof_indices, dof_values \
     		= robot.configurations.get_configuration('tucked')
     	robot.SetDOFValues(dof_values, dof_indices)

    #attaching viewer
    if attach_viewer = True:
     	env.SetViewer('qtcoin')

    prpy.logger.remove_ros_logger()
    return env,robot
	#Enable baking if it is supported.
	# try:
	# 	result = collision_checker.SendCommand('BakeGetType')
 #        is_baking_suported = (result is not None)
 #    except openrave_exception:
 #     	is_baking_suported = False
 #    if is_baking_suported:
 #    	robot_checker_factory = BakedRobotCollisionCheckerFactory()
 #    else:
 #    	robot_checker_factory = SimpleRobotCollisionCheckerFactory()
 #    	logger.warning('Collision checker does not support baking. Defaulting to'
 #    		' the slower SimpleRobotCollisionCheckerFactory.')
    

	  
 #    if sim = True:
 #    	dof_indices, dof_values \
 #    		= robot.configurations.get_configuration('tucked')
 #    	robot.SetDOFValues(dof_values, dof_indices)

 #    #attaching viewer
 #    if attach_viewer = True:
 #    	env.SetViewer('qtcoin')

 #    prpy.logger.remove_ros_logger()
 #    return env,robot





