#!/usr/bin/env python
import fetchpy
import numpy
import rospy

if __name__ == '__main__':

	rospy.init_node('fetchpy')
	env, robot = fetchpy.initialize(sim=False)
	
	keep_going = True

	while keep_going:
		print "\n\nWelcome to Pose Control:\n \
		Press 0 to dock the arm.\n \
		Press 1 to dock the arm and the torso.\n \
		Press 2 to say something.\n \
		Press 3 to straight the arm.\n \
		Press 4 to go to cleaing wall.\n \
		Press 5 to wave.\n \
		Press 6 to do I love you.\n \
		Press 7 to say Yes.\n \
		Press 8 to say No.\n \
		Press 9 to quit." 

		user_input = int(raw_input("Gesture? "))

		if user_input == 0:
			print 'Going to dock the arm!\n'
			robot.arm.PlanToNamedConfiguration('arm_dock', execute=True)
			robot.gripper.CloseHand()
		elif user_input == 1:
			print 'Going to dock the arm and the torso'
			arm_dock = [ 0.0,1.3200049175915627, 1.3999834311949337, 
			-0.19984533366667812,  1.7199679814216946, 2.70622649445329e-06 ,
			 1.6600064741153062, -1.3477542619710903e-06 ]     
			robot.arm_torso.PlanToConfiguration(arm_dock, execute = True)   
		elif user_input == 2:
			say_this = raw_input('What do you want Fetch to say? ')
			robot.Say(say_this)
		elif user_input == 3:
			print 'Straightning the arm!\n'
			robot.arm.PlanToNamedConfiguration('straight', execute=True)
			robot.gripper.OpenHand()
		elif user_input == 4:
			print 'Clean wall position!\n'
			clean_wall = ([0.35,-0.1803065,-0.71321057,0.07084394,1.23903305,-0.10976212,-0.62392059,0.0])
			robot.arm_torso.PlanToConfiguration(clean_wall, execute = True) 
		elif user_input == 5:
			print 'I am Waving!\n'
			robot.Wave2()
		elif user_input == 6:
			print 'I love you!\n'
			robot.ILOVEYOU()
		elif user_input == 7:
			print 'Doing Yes'
			robot.NodYes()
		elif user_input == 8:
			print 'Doing No'
			robot.NodNo()
		elif user_input == 9:
			print 'Goodbye!\n'
			keep_going = False


		else:
			print 'Input Not Recognized\n'


	
