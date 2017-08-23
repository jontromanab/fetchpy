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



	

