import openravepy, logging
import math
import logging
import numpy as np
from openravepy import *
import warnings

from prpy.util import *
from prpy.planning.retimer import OpenRAVEAffineRetimer, ParabolicRetimer, HauserParabolicSmoother
from prpy.rave import save_trajectory
from prpy.clone import Clone, Cloned

logger = logging.getLogger('fetchpy')

def or_traj_to_ros_vel(robot,traj):
''' Creates a consecutive set of velocities and timestamps from an openrave affine trajectory '''
	positions = []
	time = []
	cspec = traj.GetConfigurationSpecification()
	for iwaypoint in xrange(traj.GetNumWaypoints()):
		waypoint = traj.GetWaypoint(iwaypoint)
		#affdofvalues = cspec.ExtractAffineValues(waypoint, robot, 11, 1)
		if len(waypoint)<=7:
			#affdofvalues = cspec.ExtractAffineValues(waypoint, robot, 11, 1)
			affdofvalues = waypoint[0:4]
			#dt = cspec.ExtractDeltaTime(waypoint)
			dt = waypoint[6]
			time.append(dt)
		else:
			affdofvalues = waypoint[8:11]
			#dt = cspec.ExtractDeltaTime(waypoint)
			dt = waypoint[22]
			time.append(dt)
		vel_imd = []
		vel_imd.append(affdofvalues[0])
		vel_imd.append(affdofvalues[2])
		positions.append(vel_imd)
	return positions, time

def create_affine_trajectory(robot, poses):
''' Creates an untimed openrave affine trajectory from a set of affine poses '''
    doft = openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.RotationAxis
    cspec = openravepy.RaveGetAffineConfigurationSpecification(doft, robot)
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
    traj.Init(cspec)
    for iwaypoint, pose in enumerate(poses):
        waypoint = openravepy.RaveGetAffineDOFValuesFromTransform(pose, doft)
	traj.Insert(iwaypoint, waypoint)
    return traj

def create_new_base_waypoints(base_joint_values, size):
''' Creates new base waypoints from the center to given point '''
	first_point = np.array([0,0,0])
	last_point = np.array([base_joint_values[0], 0, base_joint_values[1]])
	diff = (first_point - last_point)/(size-1)
	new_wayspoints = []
	new_wayspoints.append(first_point)
	for i in range(size-1):
		first_point = first_point - diff
		new_wayspoints.append(first_point)
	return new_wayspoints

def create_new_timestamps(final_point, size):
	first_timestamp = 0.0
	diff = (final_point - first_timestamp)/(size-1)
	new_timestamps = []
	new_timestamps.append(first_timestamp)
	for i in range(size - 1):
		first_timestamp = first_timestamp +diff
		new_timestamps.append(first_timestamp)
	return new_timestamps

def higherNumber(a,b):
	if(a>b):
		return a
	else:
		return b

'''Whole Body Trajectory (arm_positions + base_positions + arm+velocities
+base_velocities[0.*3] + base_delta_time + arm_delta_time'''
def ExtractBTrajFromWholeBodyTraj(robot, traj):
	doft = openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.RotationAxis
	cspec = traj.GetConfigurationSpecification()
	bcspec = openravepy.RaveGetAffineConfigurationSpecification(doft, robot)
	base_traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
	bcspec.AddGroup('affine_velocities', dof= 3, interpolation='next')
	bcspec.AddDeltaTimeGroup()
	base_traj.Init(bcspec)
	for iwaypoint in xrange(traj.GetNumWaypoints()):
		value = []
		waypoint = traj.GetWaypoint(iwaypoint)
		value.extend(waypoint[8:11])
		value.extend(waypoint[19:23])
		base_traj.Insert(iwaypoint, value)
	return base_traj


def RetimeWholeBodyTrajectory(robot, arm_traj, base_traj):
	affine_retimer = OpenRAVEAffineRetimer()
	retimer = ParabolicRetimer()
	smoother = HauserParabolicSmoother()
	simplifier = None
	env = arm_traj.GetEnv()
	cspec = arm_traj.GetConfigurationSpecification()
	dof_indices, _ = cspec.ExtractUsedIndices(robot)
	affine_dofs = (DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
	#_postprocess_envs = collections.defaultdict(openravepy.Environment)
	'''Separating base and arm trajectory and try to retime them individually.
	As openrave does not support retime both of the affine and joint_dof at same time
	There will be issues.As affine retiming keeps the same number of points while
	joint_dof retiming reduces number of points.'''
	postprocess_envs = robot._postprocess_envs[openravepy.RaveGetEnvironmentId(env)]
	retiming_options = dict()
	affine_retimer_options = dict()
	with Clone(env, clone_env = postprocess_envs) as cloned_env:
		cloned_robot1 = cloned_env.Cloned(robot)
		cloned_robot1.SetActiveDOFs(dof_indices)
		arm_timed_traj = retimer.RetimeTrajectory(cloned_robot1, arm_traj, **retiming_options)
		arm_final_traj = CopyTrajectory(arm_timed_traj, env=env)

		cloned_robot2 = cloned_env.Cloned(robot)
		cloned_robot2.SetActiveDOFs([],affine_dofs)
		base_timed_traj = affine_retimer.RetimeTrajectory(cloned_robot2, base_traj, **affine_retimer_options)
		base_final_traj = CopyTrajectory(base_timed_traj, env = env)
	'''Creating a whole body trajectory from arm_timed and base_timed traj
	 The timestamp and number of waypoints are going to be different from both the trajectories
	 so if the arm_timed_traj has higher number of waypoints, after the waypoints from
	 base are over, every other waypoint will receive zeros for position and velocity and if the 
	 base_timed_traj has higher number of waypoints, then all the arm waypoints will receive 
	 the values of last waypoint. Openrave trajectory format now only can receive one deltatime.
	 (Hacky way) as we dont use the affine velocity, the base time is merged in affine_velocity. 
	 Thats why it is 4'''

	base_num_of_way = base_final_traj.GetNumWaypoints()
	arm_num_of_way = arm_final_traj.GetNumWaypoints()
	print 'base has '+str(base_num_of_way)+' waypoints'
	print 'arm has '+str(arm_num_of_way)+' waypoints'
	a_cspec = arm_final_traj.GetConfigurationSpecification()
	b_cspec = base_final_traj.GetConfigurationSpecification()

	with env:
		robot.SetActiveDOFs(dof_indices, affine_dofs)
		cspec = robot.GetActiveConfigurationSpecification('linear')
		traj = openravepy.RaveCreateTrajectory(env, '')
		cspec.AddGroup('joint_velocities', dof= 8, interpolation='quadratic')
		cspec.AddGroup('affine_velocities', dof= 4, interpolation='next')
		cspec.AddDeltaTimeGroup()
		traj.Init(cspec)

		highest_way = higherNumber(base_num_of_way, arm_num_of_way)
		arm_final_waypoint  = []
		for i in range(highest_way):
			value = []
			arm_waypoint = []
			base_waypoint = []
			dt = 0.0
			if(i>=base_num_of_way):
				arm_waypoint = arm_final_traj.GetWaypoint(i)
				base_waypoint = np.zeros(7)
				dt = arm_waypoint[-1]
				
			elif(i>=arm_num_of_way):
				arm_waypoint = arm_final_waypoint
				base_waypoint = base_final_traj.GetWaypoint(i)
				dt = base_waypoint[-1]
			else:
				arm_waypoint = arm_final_traj.GetWaypoint(i)
				base_waypoint = base_final_traj.GetWaypoint(i)
				dt = arm_waypoint[-1]

			arm_way_point = arm_waypoint[:-1]
			value.extend(arm_way_point[:8])
			value.extend(base_waypoint[:3])
			value.extend(arm_way_point[8:])
			value.extend(base_waypoint[3:])
			value.extend([dt])
			traj.Insert(i, value)
		 	arm_final_waypoint = arm_waypoint
	return traj

def create_whole_body_trajectory(robot, arm_traj, base_traj):
	env = arm_traj.GetEnv()
	whole_timed_traj = RetimeWholeBodyTrajectory(robot, arm_traj, base_traj)
	save_trajectory(whole_timed_traj,'/home/abhi/Desktop/traj2/whole_body_timed_traj.xml')
	#save_trajectory(arm_timed_traj,'/home/abhi/Desktop/traj2/arm_timed_traj.xml')
	#save_trajectory(base_timed_traj,'/home/abhi/Desktop/traj2/base_timed_traj.xml')
	return whole_timed_traj


