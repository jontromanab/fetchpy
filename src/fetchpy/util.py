import openravepy, logging
import math
import logging
import numpy as np
from openravepy import *
import warnings

from prpy.util import *
from prpy.planning.retimer import OpenRAVEAffineRetimer, ParabolicRetimer, HauserParabolicSmoother
from prpy.rave import save_trajectory

logger = logging.getLogger('fetchpy')

def or_traj_to_ros_vel(robot,traj):
	positions = []
	time = []
	cspec = traj.GetConfigurationSpecification()
	for iwaypoint in xrange(traj.GetNumWaypoints()):
		waypoint = traj.GetWaypoint(iwaypoint)
		#affdofvalues = cspec.ExtractAffineValues(waypoint, robot, 11, 1)
		if len(waypoint)<=7:
			affdofvalues = cspec.ExtractAffineValues(waypoint, robot, 11, 1)
			dt = cspec.ExtractDeltaTime(waypoint)
			time.append(dt)
		else:
			affdofvalues = waypoint[16:19]

		vel_imd = []
		vel_imd.append(affdofvalues[0])
		vel_imd.append(affdofvalues[2])
		#print vel_imd
		positions.append(vel_imd)
	return positions, time

def create_affine_trajectory(robot, poses):
    doft = openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.RotationAxis
    cspec = openravepy.RaveGetAffineConfigurationSpecification(doft, robot)
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), 'GenericTrajectory')
    traj.Init(cspec)

    for iwaypoint, pose in enumerate(poses):
        waypoint = openravepy.RaveGetAffineDOFValuesFromTransform(pose, doft)
	traj.Insert(iwaypoint, waypoint)
    return traj

def create_new_base_waypoints(base_joint_values, size):
	first_point = np.array([0,0,0])
	last_point = np.array([base_joint_values[0], 0, base_joint_values[1]])
	diff = (first_point - last_point)/(size-1)
	#diff = [(a - b)/(size-1) for a, b in zip(first_point, last_point)]
	new_wayspoints = []
	new_wayspoints.append(first_point)
	for i in range(size-1):
		first_point = first_point - diff
		new_wayspoints.append(first_point)
	return new_wayspoints


def RetimeWholeBodyTrajectory(robot, untimed_traj, base_joint_values):
	affine_retimer = OpenRAVEAffineRetimer()
	retimer = ParabolicRetimer()
	smoother = HauserParabolicSmoother()
	simplifier = None
	env = untimed_traj.GetEnv()
	cspec = untimed_traj.GetConfigurationSpecification()
	dof_indices, _ = cspec.ExtractUsedIndices(robot)
	affine_dofs = (DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis)
	'''Separating base and arm trajectory and try to retime them individually.
	As openrave does not support retime both of the affine and joint_dof at same time
	There will be issues.As affine retiming keeps the same number of points while
	joint_dof retiming reduces number of points.'''
	#base_waypoints = []
	with env:
		robot.SetActiveDOFs(dof_indices)
		arm_traj = openravepy.RaveCreateTrajectory(env, '')
		acspec = robot.GetActiveConfigurationSpecification('linear')
		arm_traj.Init(acspec)
		for i in range(untimed_traj.GetNumWaypoints()):
			value = untimed_traj.GetWaypoint(i)[:8]
			arm_traj.Insert(i, value)
			
		retiming_options = dict()
		smoothing_options = dict()
		
		arm_timed_traj = retimer.RetimeTrajectory(
                        robot, arm_traj, **retiming_options)
        
        save_trajectory(arm_timed_traj,'/home/abhi/Desktop/traj2/arm_timed_traj.xml')
        time = []
        a_cspec = arm_timed_traj.GetConfigurationSpecification()
        
        new_base_points = create_new_base_waypoints(base_joint_values, arm_timed_traj.GetNumWaypoints())
    	#Creating new trajectory
    	robot.SetActiveDOFs(dof_indices, affine_dofs)
    	cspec = robot.GetActiveConfigurationSpecification('linear')
    	traj = openravepy.RaveCreateTrajectory(env, '')
    	cspec.AddGroup('joint_velocities', dof= 8, interpolation='quadratic')
    	#cspec.AddGroup('affine_velocities', dof= 3, interpolation='next')
    	cspec.AddDeltaTimeGroup()
    	traj.Init(cspec)
    	for i in range(arm_timed_traj.GetNumWaypoints()):
    		value = []
    		waypoint = arm_timed_traj.GetWaypoint(i)
    		way = waypoint[:-1]
    		value.extend(way)
    		value.extend(new_base_points[i])
    		dt = a_cspec.ExtractDeltaTime(waypoint)
    		value.extend([dt])
    		traj.Insert(i, value)

	return traj

def create_whole_body_trajectory(robot, arm_traj, base_joint_values):
	env = arm_traj.GetEnv()
	timed_traj = RetimeWholeBodyTrajectory(robot, arm_traj, base_joint_values)
	return timed_traj


