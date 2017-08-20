PACKAGE = 'fetchpy'
import logging
import numbers
import prpy
import prpy.rave
import prpy.util
import yaml
import subprocess

#all the components of the robot
from .arm import ARM
from .gripper import GRIPPER

from prpy import Cloned
from prpy.action import ActionLibrary
from prpy.base.robot import Robot
from prpy.controllers import RewdOrTrajectoryController
from prpy.exceptions import TrajectoryNotExecutable
from prpy.named_config import ConfigurationLibrary
from prpy.planning import (
    CBiRRTPlanner,
    FirstSupported,
    NamedPlanner,
    SBPLPlanner,
    Sequence,
    SnapPlanner,
    TSRPlanner,
    OMPLPlanner,
    VectorFieldPlanner,
)

from or_trajopt import TrajoptPlanner
from prpy.planning.retimer import HauserParabolicSmoother
from prpy.util import FindCatkinResource

logger = logging.getLogger('fetchpy')

def try_and_warn(fn, exception_type, message, default_value=None):
    try:
        return fn()
    except exception_type:
        logger.warning(message)
        return None



class FETCHRobot(Robot):
    def __init__(self,arm_sim,arm_torso_sim,
        gripper_sim, head_sim, torso_sim, base_sim, talker_sim, 
        perception_sim,robot_checker_factory):

        Robot.__init__(self, robot_name = 'fetch')
        self.robot_checker_factory = robot_checker_factory


        #Controller Setup
        self.controller_manager = None
        self.controller_always_on = []
        self.full_controller_sim = (arm_sim and arm_torso_sim and 
        gripper_sim and head_sim)

        if not self.full_controller_sim:
            #print ' I am HERE'
            import rospy
            from ros_control_client_py import(ControllerManagerClient,
                JointStateClient,)
            import rosgraph.masterapi 

            if not rospy.core.is_initialized():
                raise RuntimeError('rospy not initialized.'
                    'Must call rospy.init_node()')

            master = rosgraph.masterapi.Master('/rostopic') 
            topics = []
            for i in master.getPublishedTopics(''):
                topics.append(i[0])
            
            if '/joint_states' not in topics:
                raise RuntimeError('The -sim argument should be set if there is'
                    ' No real robot/Gazebo robot connected')

            
            #update openrave state from /joint_states
            self._jointstate_client = JointStateClient(self, topic_name='/joint_states')
            #self.controller_manager = ControllerManagerClient()
            self.controller_always_on.append('joint_state_controller')

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('arm')
        self.arm_torso = self.GetManipulator('arm_torso')
        self.gripper = self.arm.GetEndEffector()
        self.hand = self.gripper
        #####ADD REST HERE (change)
        self.manipulators = [self.arm, self.arm_torso]

        #Dynamically switch to self-specific subclasses.
        prpy.bind_subclass(self.arm, ARM, sim= arm_sim)
        prpy.bind_subclass(self.gripper, GRIPPER, sim = gripper_sim, manipulator = self.arm,
            namespace = '')
        #####ADD REST HERE(change)

        # Set FETCH's acceleration limits. These are not specified in URDF.
        accel_limits = self.GetDOFAccelerationLimits()
        accel_limits[self.arm.GetArmIndices()] = [2.] * self.arm.GetArmDOF()
        #get acceleration limits and set them here(change)
        self.SetDOFAccelerationLimits(accel_limits)


        # Determine always-on controllers(have to do all the controllers here)
        #arm controller
        # Set default manipulator controllers in sim only
        if not gripper_sim:
            self.controller_always_on.append('gripper_controller')
        if arm_sim:
            self.arm.sim_controller = self.AttachController(name=self.arm.GetName(),
                args = 'IdealController', dof_indices = self.arm.GetArmIndices(),
                affine_dofs=0, simulated=True)
            
        #load and activate initial controllers(change)
        #if self.controller_manager is not None:
            #self.controller_manager.request(self.controller_always_on).switch()

        # Support for named configurations.(change)
        import os.path
        self.configurations.add_group('arm',self.arm.GetArmIndices())

        configurations_path = FindCatkinResource('fetchpy', 'config/configurations.yaml')

        #(change)
        try: 
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            raise ValueError('Failed loading named configurations from "{:s}".'.format(
                configurations_path))

        #Hand configurations
        #SOME OF THEM LIKE OPEN CLOSE GRASPING ETC. todo NOW(change)
        self.gripper.configurations = ConfigurationLibrary()
        self.gripper.configurations.add_group('gripper', self.gripper.GetIndices())
        # if isinstance(self.hand, GRIPPER):
        #     gripper_configurations_path = FindCatkinResource('fetchpy', 'config/gripper_preshapes.yaml')
        #     try:
        #         self.configurations.load_yaml(gripper_configurations_path)
        #     except IOError as e:
        #         raise ValueError('Failed loading named configurations from "{:s}".'.format(
        #             gripper_configurations_path))



        #Planner.
        snap_planner = SnapPlanner(robot_checker_factory=self.robot_checker_factory)

        vectorfield_planner = VectorFieldPlanner(robot_checker_factory=self.robot_checker_factory)
        trajopt_planner = TrajoptPlanner(
            robot_checker_factory=self.robot_checker_factory)
        rrt_planner = OMPLPlanner('RRTConnect',
            robot_checker_factory=self.robot_checker_factory)
        cbirrt_planner = CBiRRTPlanner(
            timelimit=1.,
            robot_checker_factory=self.robot_checker_factory)

        actual_planner = Sequence(
            snap_planner,
            vectorfield_planner,
            trajopt_planner,
            TSRPlanner(
                delegate_planner=Sequence(snap_planner, trajopt_planner),
                robot_checker_factory=self.robot_checker_factory),
            FirstSupported(
                rrt_planner,
                cbirrt_planner),
        )

        self.planner = FirstSupported(
            actual_planner,
            NamedPlanner(delegate_planner=actual_planner),
        )


        # Post-processor.
        self.smoother = HauserParabolicSmoother(
            do_blend=True, blend_iterations=1, blend_radius=0.4,
            do_shortcut=True, timelimit=0.6)
        self.retimer = HauserParabolicSmoother(
            do_blend=True, blend_iterations=1, blend_radius=0.4,
            do_shortcut=False)
        self.simplifier = None

        #Base Planning
        #DO WE ACTUALLY NEED THIS? HOPEFULLY NOT(change)


        #Create action library(change)
        self.actions = ActionLibrary()

        # Register default actions and TSRs: TODO(change)
        #import fetchpy.action
        #import fetch.tsr

        #Setting necessary sim flags
        self.talker_simulated = talker_sim
        self.base_sim = base_sim


        #Set up perception(change)
        self.detector = None
        if perception_sim:
            from prpy.perception import SimulatedPerceptionModule
            self.detector = SimulatedPerceptionModule()
        else:
        	from prpy.perception import ApriltagsModule
        	try:
        		kinbody_path = FindCatkinResource('pr_ordata',
                                                            'data/objects')
        		marker_data_path = FindCatkinResource('pr_ordata',
                                                                'data/objects/tag_data.json')
        		self.detector = ApriltagsModule(marker_topic='/apriltags_kinect2/marker_array',
        			marker_data_path=marker_data_path,kinbody_path=kinbody_path,
        			detection_frame='head/kinect2_rgb_optical_frame',
        			destination_frame='fetch_base',reference_link=self.GetLink('/base'))
        	except IOError as e:
        		logger.warning('Failed to find required resource path. ' \
                               'pr_ordata package cannot be found. ' \
                               'Perception detector will not be loaded.' \
                               '\n{}'.format(e))
        #HAVE TO IMPLEMENT TALKER(todo)(change)
        # if not self.talker_simulated:
        #     # Initialize herbpy ROS Node
        #     import rospy
        #     if not rospy.core.is_initialized():
        #         raise RuntimeError('rospy not initialized. '
        #                            'Must call rospy.init_node()')

        #     import talker.msg
        #     from actionlib import SimpleActionClient
        #     self._say_action_client = SimpleActionClient('say', talker.msg.SayAction)


    def CloneBindings(self, parent):
        super(FETCHRobot, self).CloneBindings(parent)
        self.arm = Cloned(parent.arm)
        self.manipulators = [self.arm]
        self.gripper = Cloned(parent.arm.GetEndEffector())
        self.hand = self.gripper
        self.planner = parent.planner

    	#Add all here (change)

    def _ExecuteTrajectory(self, traj, defer=False, timeout=None, period=0.01,**kwargs):
        #print 'HELLLLLLLLOOOOOOOOOOOOOO>>>>>>>I am here<<<<<<<<<'
        if defer is not False:
            raise RuntimeError('defer functionality was deprecated in ''personalrobotics/prpy#278')
            # Don't execute trajectories that don't have at least one waypoint.
        if traj.GetNumWaypoints() <= 0:
            raise ValueError('Trajectory must contain at least one waypoint.')
        # Check if this trajectory contains both affine and joint DOFs
        cspec = traj.GetConfigurationSpecification()
        needs_base = prpy.util.HasAffineDOFs(cspec)
        needs_joints = prpy.util.HasJointDOFs(cspec)
        if needs_base and needs_joints:
            raise ValueError('Trajectories with affine and joint DOFs are not supported')

        # Check that the current configuration of the robot matches the
        # initial configuration specified by the trajectory.
        if not prpy.util.IsAtTrajectoryStart(self, traj):
            raise TrajectoryNotExecutable('Trajectory started from different configuration than robot.')
        # If there was only one waypoint, at this point we are done!
        if traj.GetNumWaypoints() == 1:
            return traj
        # Verify that the trajectory is timed by checking whether the first
        # waypoint has a valid deltatime value.
        #if not prpy.util.IsTimedTrajectory(traj):
            #raise ValueError('Trajectory cannot be executed, it is not timed.')

        # Verify that the trajectory has non-zero duration.
        if traj.GetDuration() <= 0.0:
            logger.warning('Executing zero-length trajectory. Please update the'
                          ' function that produced this trajectory to return a'
                          ' single-waypoint trajectory.', FutureWarning)
        

        traj_manipulators = self.GetTrajectoryManipulators(traj)
        controllers_manip = []
        active_controllers = []

        #HEAD in traj(change)
        # if self.head in traj_manipulators:
        # 	# TODO head after Schunk integration
        #     if len(traj_manipulators) == 1:
        #         raise NotImplementedError('The head is currently disabled under ros_control.')
        #     else:
        #         logger.warning('The head is currently disabled under ros_control.')

        # logic to determine which controllers are needed(change)
        if self.arm in traj_manipulators:
            if not self.arm.IsSimulated():
                controllers_manip.append('arm_controller')
            else:
                active_controllers.append(self.arm.sim_controller)
    

        # load and activate controllers(change)
        #if not self.full_controller_sim:
        #self.controller_manager.request(controllers_manip).switch()

        # repeat logic and actually construct controller clients
        # now that we've activated them on the robot
        if 'arm_controller' in controllers_manip:
            active_controllers.append(RewdOrTrajectoryController(self, '',
                'arm_controller',self.arm.GetJointNames()))

        ##ADD HERE ALL THE CONTROLLERS (change)
        for controller in active_controllers:
	        controller.SetPath(traj)

        prpy.util.WaitForControllers(active_controllers, timeout=timeout)
        return traj


    def ExecuteTrajectory(self, traj, *args, **kwargs):
        #print 'HELLLLLLLLOOOOOOOOOOOOOO>>>>>>>I am here executing<<<<<<<<<'
        value = self._ExecuteTrajectory(traj, *args, **kwargs)
        return value

        # Inherit docstring from the parent class.
        ExecuteTrajectory.__doc__ = Robot.ExecuteTrajectory.__doc__


        def SetStiffness(self, stiffness, manip=None):
            """Set the stiffness of HERB's arms and head.
            Stiffness False/0 is gravity compensation and stiffness True/(>0) is position
            control.
            @param stiffness boolean or numeric value 0.0 to 1.0
            """
            raise NotImplementedError('Not Implemented yet AGAIN.'
                'There is a gravity compensation controller on real robot. Just subscribe to that and enable/disable')


            # if (isinstance(stiffness, numbers.Number) and
            #         not (0 <= stiffness and stiffness <= 1)):
            #     raise Exception('Stiffness must be boolean or numeric in the range [0, 1];'
            #                     'got {}.'.format(stiffness))

            # # TODO head after Schunk integration
            # if manip is self.head:
            #     raise NotImplementedError('Head immobilized under ros_control, SetStiffness not available.')

            # new_manip_controllers = []
            # if stiffness:
            #     if not self.left_arm.IsSimulated() and (manip is None or manip is self.left_arm):
            #         new_manip_controllers.append('left_joint_group_position_controller')
            #     if not self.right_arm.IsSimulated() and (manip is None or manip is self.right_arm):
            #         new_manip_controllers.append('right_joint_group_position_controller')
            # else:
            #     if not self.left_arm.IsSimulated() and (manip is None or manip is self.left_arm):
            #         new_manip_controllers.append(
            #             'left_gravity_compensation_controller')
            #     if not self.right_arm.IsSimulated() and (manip is None or manip is self.right_arm):
            #         new_manip_controllers.append(
            #             'right_gravity_compensation_controller')

            # if not self.full_controller_sim:
            #     self.controller_manager.request(new_manip_controllers).switch()

            ####(change)




        def Say(self, words, block=True):
            """Speak 'words' using talker action service or espeak locally in simulation"""
            raise NotImplementedError('Not Implemented yet AGAIN.'
                'There is a gravity compensation controller on real robot. Just subscribe to that and enable/disable')
            ####(change)
            # if self.talker_simulated:
            #     try:
            #         proc = subprocess.Popen(['espeak', '-s', '160', '"{0}"'.format(words)])
            #         if block:
            #             proc.wait()
            #     except OSError as e:
            #         logger.error('Unable to speak. Make sure "espeak" is installed locally.\n%s' % str(e))
            # else:
            #     import talker.msg
            #     goal = talker.msg.SayGoal(text=words)
            #     self._say_action_client.send_goal(goal)
            #     if block:
            #         self._say_action_client.wait_for_result()
            























