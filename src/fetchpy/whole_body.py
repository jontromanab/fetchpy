import numpy, openravepy, logging
import prpy, time, rospy
from openravepy import *
import actionlib

from .head import FollowJointTrajectoryController
from .base import BaseVelocityController

from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (Twist, Vector3)
import util

from rospy import Duration

logger = logging.getLogger('fetchpy')

from .base import BaseVelocityPublisher

class WholeBodyController(object):
	def __init__(self, ns, robot, sim = False, timeout = 0.0):
		self.logger = logging.getLogger(__name__)
		self.robot = robot
		self.client_ = actionlib.SimpleActionClient("/arm_with_torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction,)
		self.Publisher = BaseVelocityPublisher(ns, 'base_controller', timeout)
		#self.pub_ = rospy.Publisher('/base_controller/command', Twist, queue_size=1)
		self._current_cmd = None

	def createArmTrajectory(self, values):
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration.from_sec(0.0)
		traj_msg.joint_names = self.GetJointNames()
		traj_msg.points = []
		end_point = JointTrajectoryPoint()
		end_point.positions = values
		end_point.velocities = [0,0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		end_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		end_point.time_from_start = rospy.Duration.from_sec(1.0)
		traj_msg.points.append(end_point)
		return traj_msg

	def getJointState(self):
		self.SetActive()
		values =  self.robot.GetActiveDOFIndices()
		return values

	def SetActive(self):
		self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.GetJointNames()])

	def GetJointNames(self):
		jointnames = ['torso_lift_joint','shoulder_pan_joint',
		'shoulder_lift_joint',
		'upperarm_roll_joint',
		'elbow_flex_joint',
		'forearm_roll_joint',
		'wrist_flex_joint',
		'wrist_roll_joint']
		return jointnames

	def SetPath(self, traj):
		cspec = traj.GetConfigurationSpecification()
		dof_indices, _ = cspec.ExtractUsedIndices(self.robot)
		time_from_start = 0.
		prev_time_from_start = 0.
		traj_msg = JointTrajectory()
		traj_msg.joint_names = self.GetJointNames()
		for iwaypoint in xrange(traj.GetNumWaypoints()):
			waypoint = traj.GetWaypoint(iwaypoint)
			q = waypoint[:8]
			qd = waypoint[11:19]
			qdd = [0.] * 8
			dt = waypoint[-1]
			time_from_start += dt
			deltatime = time_from_start - prev_time_from_start
			prev_time_from_start = time_from_start
			traj_msg.points.append(
            JointTrajectoryPoint(
                positions=list(q),
                velocities=list(qd) if qd is not None else [],
                accelerations=list(qdd) if qdd is not None else [],
                time_from_start=Duration.from_sec(time_from_start)
            )
        )
		goal_msg = FollowJointTrajectoryGoal()
		goal_msg.trajectory = traj_msg
		
		positions,times = util.or_traj_to_ros_vel(self.robot, traj)
		self.client_.send_goal(goal_msg)
		curr_pos= positions[0]
		curr_time = times[0]
		
		for i in range(len(positions)-1):
			final_base_goal = [a - b for a, b in zip(positions[i+1], curr_pos)]
			final_time_goal = times[i+1] - curr_time
			final_vel_goal = [j/final_time_goal for j in final_base_goal]
			#self.logger.info('Moving by: {}'.format(final_vel_goal))
			self._current_cmd = self.Publisher.execute(final_vel_goal,final_time_goal) 
			curr_pos = positions[i+1]
			curr_time = times[i+1]
		#self.Publisher.executeTraj(positions)
		
		


	def IsDone(self):
		return self._current_cmd is None or self._current_cmd.done()

	def execute(self, values):
 		#print 'I am executing: '+str(values)
		goal_msg_base = Twist()
		vel = values[-2:]
		goal_msg_base.linear.x = vel[0]
		goal_msg_base.angular.z = vel[1]
		#print goal_msg_base
		arm_traj = self.createArmTrajectory(values[:-2])
		goal_msg = FollowJointTrajectoryGoal()
		goal_msg.trajectory = arm_traj
		self.Publisher.execute(vel)
		self.client_.send_goal(goal_msg)
		#self.client_.wait_for_result()
		#self.pub_.publish(goal_msg_base)
		#self.Publisher.execute(vel)








class WholeBody():
	def __init__(self, sim, robot):
		self.simulated = sim
		self.logger = logging.getLogger(__name__)
		self.robot = robot
		self.controller = WholeBodyController('',self.robot,sim)
		
	def IsSimulated(self):
		return self.simulated


	def PlanToConfiguration(self, values, execute = False, timeout = None, **kwargs):
		from prpy.rave import save_trajectory
		arm_joint_values = values[:8]
		base_affine_values = values[-2:]
		arm_traj = self.robot.arm_torso.PlanToConfiguration(arm_joint_values)
		base_traj = self.robot.base.Move(base_affine_values)
		#self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in self.controller.GetJointNames()],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis)
		traj = util.create_whole_body_trajectory(self.robot, arm_traj, base_traj)
		if(execute):
			self.robot.ExecuteTrajectory(traj, **kwargs)
		else:
			return traj

	def Move(self, joint_values):
		self.controller.execute(joint_values)











