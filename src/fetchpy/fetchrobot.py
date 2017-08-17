PACKAGE = 'fetchpy'
import logging
import numbers
import prpy
import prpy.rave
import prpy.util
import yaml
import subprocess

#### all the parts of fetch

from prpy import Cloned
from prpy.base.robot import Robot
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

def try_and_warn(fn, exeception_type, message, default_value = None):
	try:
		return fn()
	except exception_type:
		logger.warning(message)
		return None

class FETCHRobot(Robot):
	def __init__(self, robot_checker_factory):
		Robot.__init__(self,robot_name = 'fetch')
		self.robot_checker_factory = robot_checker_factory

	