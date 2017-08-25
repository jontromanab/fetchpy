import logging, openravepy, prpy
from prpy.action import ActionMethod
from prpy.util import FindCatkinResource, GetPointFrom
import numpy, time

from prpy.rave import load_trajectory
from os.path import join


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
	env = robot.GetEnv()
	wave_path = FindCatkinResource('fetchpy', 'config/WaveTraj/')
	traj0 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj1 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	traj2 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj3 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	manip = robot.arm
	robot.HaltHand(manip = manip)
	pause = 0.30
	time.sleep(pause)
	robot.ExecuteTrajectory(traj0)
	robot.ExecuteTrajectory(traj1)
	robot.ExecuteTrajectory(traj2)
	robot.ExecuteTrajectory(traj3)

@ActionMethod
def Wave2(robot):
	robot.gripper.OpenHand()
	pose1 = ([-1.50, 0.55, 0.00, -1.94, 0.00, 0.00, -1.43])
	robot.arm.PlanToConfiguration(pose1, execute = True)
	env = robot.GetEnv()
	wave_path = FindCatkinResource('fetchpy', 'config/WaveTraj2/')
	traj0 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj1 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	traj2 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj3 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	traj4 = load_trajectory(env, join(wave_path, 'wave1.xml'))
	traj5 = load_trajectory(env, join(wave_path, 'wave2.xml'))
	manip = robot.arm
	pause = 0.60
	time.sleep(pause)
	robot.ExecuteTrajectory(traj1)
	robot.ExecuteTrajectory(traj2)
	robot.ExecuteTrajectory(traj3)
	robot.ExecuteTrajectory(traj4)
	robot.ExecuteTrajectory(traj5)
	robot.ExecuteTrajectory(traj0)
	robot.gripper.CloseHand()

	

@ActionMethod
def ILOVEYOU(robot):
	robot.gripper.CloseHand()
	env = robot.GetEnv()
	i_pattern = ([-1.03, 0.2, -1.54, -1.52, -0.05, -2.14, 0.09])
	robot.arm.PlanToConfiguration(i_pattern, execute = True) 
	robot.Say('I')
	l_pattern = ([-1.49, -0.00, 0.00, -1.65, 0.00, 0.00, 0.00])
	robot.arm.PlanToConfiguration(l_pattern, execute = True)
	robot.Say('love')
	u_pattern = ([-0.1803065,-0.71321057,0.07084394,1.23903305,-0.10976212,-0.62392059,0.0]) 
	robot.arm.PlanToConfiguration(u_pattern, execute = True)
	robot.Say('youu')

@ActionMethod
def NodYes(robot):
	pause = 0.15
	robot.Say('Yes')
	robot.head.MoveToNamedConfiguration('look_straight')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_down')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_up')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_down')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_straight')
	time.sleep(pause)

@ActionMethod
def NodNo(robot):
	pause = 0.15
	robot.Say('No')
	robot.head.MoveToNamedConfiguration('look_straight')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_right')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_left')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_right')
	time.sleep(pause)
	robot.head.MoveToNamedConfiguration('look_straight')
	time.sleep(pause)











	

