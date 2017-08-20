#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for
using fetchpy and openravepy.
"""

import os
if os.environ.get('ROS_DISTRO', 'hydro')[0] <= 'f':
    import roslib
    roslib.load_manifest('fetchpy')

import argparse, fetchpy, openravepy


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='utility script for loading HerbPy')
	parser.add_argument('-s', '--sim', action='store_true',help='simulation mode')
	# parser.add_argument('-v', '--viewer', nargs='?', const=True,
 #                        help='attach a viewer of the specified type')
 	parser.add_argument('--viewer', type=str, 
                        help='attach a viewer of the specified type')
	parser.add_argument('--robot-xml', type=str,
                        help='robot XML file; defaults to herb_description')
	parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
	parser.add_argument('-b', '--base-sim', action='store_true',
                        help='simulate base')
	parser.add_argument('-p', '--perception-sim', action='store_true',
                        help='simulate perception')
	parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
	args = parser.parse_args()

	openravepy.RaveInitialize(True)
	openravepy.misc.InitOpenRAVELogging()

	if args.debug:
		openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

	
	fetchpy_args = {'sim':args.sim,'viewer':args.viewer,'robot_xml':args.robot_xml,
	'env_path':args.env_xml,'base_sim':args.base_sim,'perception_sim': args.perception_sim}

	if not args.sim:
	    import rospy
            rospy.init_node('fetchpy')

        if args.sim and not args.base_sim:
    	    fetchpy_args['base_sim'] = args.sim

        env, robot = fetchpy.initialize(**fetchpy_args)

        import IPython
        IPython.embed()



