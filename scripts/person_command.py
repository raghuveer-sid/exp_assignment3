#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

# Initialize status as normal
status = "normal"

def callback(data):
    """ callback to get the status received from the state machine
    """
    global status
    status = data.data

def main():
	"""
	Initially is made to take commands from a person but not completed

	'play' will make the robot play with the human
	'stop' will make the robot to stop playing

	Publishers:
		pub: publishes (std_msgs.String) to /command 

	Subscribers:
		sub: subscribes (std_msgs.String) to /status

	"""
	rospy.init_node('person_command')

	global status

	time.sleep(20)
	# Publishers and subscribers
	pub = rospy.Publisher('/command', String, queue_size=10)	
	sub = rospy.Subscriber('/status', String, callback)

	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		if status == "normal":
				txt = "play"
				status = "play"
				pub.publish(txt)

			else:
				txt = "sleep"
				status = "sleep"
				pub.publish(txt)
				continue

		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	main()
