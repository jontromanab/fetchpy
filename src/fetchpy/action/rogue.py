import logging, openravepy, prpy
from prpy.action import ActionMethod
from prpy.util import FindCatkinResource, GetPointFrom
import numpy, time


logger = logging.getLogger('fetchpy')

@ActionMethod
def PointAt(robot, focus, manip = None, render = False):
	pointing_coord = GetPointFrom(focus)
	return Point(robot, pointing_coord, manip, render)

def Point(robot, coord, manip = None, render = False):
	if manip is None:
		manip = robot.GetActiveManipulator()

	focus_trns = numpy.eye(4, dtype = 'float')
	focus_trns[0:3, 3] = coord

	with robot.getEnv():
		point_tsr = robot.tsrlibrary(None, 'point', focus_trans, manip)

	p = openravepy.KinBody.SaveParameters
	with robot.CreateRobotStateSaver(p.ActiveManipulator | p.ActiveDOF):
		robot.SetActiveManipulator(manip)
		robot.SetActiveDOFs(manip.GetArmIndices())
		with prpy.viz.RenderTSRList(point_tsr, robot.GetEnv(), render=render):
			robot.PlanToTSR(point_tsr, execute=True)
	robot.gripper.CloseHand()

@ActionMethod
def HaltHand(robot, manip = None):
	if manip == None:
		manip = robot.GetActiveManipulator()
	pose = ([-0.46, -0.17, 0.0, -1.57, -0.35, 0.56, 0.0])
	print manip.GetName()
	if manip.GetName() == 'arm':
		robot.arm.PlanToConfiguration(pose, execute = True)
	else:
		pose.insert(0, 0.25)
		robot.arm_torso.PlanToConfiguration(pose, execute = True)

@ActionMethod
def Wave(robot):
	from prpy.rave import load_trajectory
	from os.path import join
	env = robot.GetEnv()

	wave_path = FindCatkinResource('fetchpy', 'config/WaveTraj/')
	traj0 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj1 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	traj2 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj3 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	manip = robot.arm
	robot.HaltHand(manip = manip)
	robot.ExecuteTrajectory(traj0)
	robot.ExecuteTrajectory(traj1)
	robot.ExecuteTrajectory(traj2)
	robot.ExecuteTrajectory(traj3)





	

