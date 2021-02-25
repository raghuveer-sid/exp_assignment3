#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String, Bool
import random
from geometry_msgs.msg import Point
from tf import transformations
import math
import actionlib
import actionlib.msg
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# define state NORMAL
class Normal(smach.State):
    """ Class for the NORMAL state

    In Normal state the dog is just moving arount in the laypput and waiting for the persons command

    State Machine:
    	NORMAL to SLEEP 

    Attributes:
    	normal_counter: (int)
    	coords: (MoveBaseGoal())
	good_coord: (bool)

    Publishers:
	pub: publishes (std_msgs.String) to /status

    Subscribers:
    	sub_command: subscriber (std_msgs.String) to /command

    Actions:
    	act_c: Client for action /move_base
		calls the action to move the robot to the specified coordinates
	
		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose
	
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['play','sleep'])
               
        #Publishers and subscribers
        self.sub_command = rospy.Subscriber('/command', String, cb_command)
	self.pub = rospy.Publisher('/status', String, queue_size=10)
	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

        # Initializations
        self.normal_counter = 1
	self.coords = MoveBaseGoal() 
	self.good_coord = False

    def execute(self, userdata):

        global sm_command

        # Restart the counter every time
        self.normal_counter = 1
        time.sleep(1)
        rospy.loginfo('Executing state NORMAL')

        # Check if there is previously a command in the buffer
        if sm_command == "play":
            print(sm_command)
            sm_command = None
            return 'play'

        # If not, proceed to randomly walk
        else:
	    #Let know to the command_recog node we can accept inputs
	    self.pub.publish('normal')
	    
	    # Amount of random walks before sleeping
            normal_times = random.randrange(1,5)

	    # Pick a good coordinate to go to
	    self.good_coord = False
	    while(self.good_coord == False):
            	x = random.randrange(-6,6)
            	y = random.randrange(-8,8)
	    	z = 0
		
		#Check if the random coordinates are reachable in the map
		if x >= 1 and y >= 4:
			self.good_coord = False
		elif x <= -1 and y <= -6:
			self.good_coord = False
		elif x >= 1 and x <=2 and y >= 1 and y <= 3:
			self.good_coord = False
		else:
			self.good_coord = True

            normal_coord = Point(x = x, y = y, z = z)

	    self.coords.target_pose.pose.position.x = normal_coord.x
	    self.coords.target_pose.pose.position.y = normal_coord.y
	    self.coords.target_pose.pose.position.z = normal_coord.z

	    self.coords.target_pose.header.frame_id = "map"
    	    self.coords.target_pose.pose.orientation.w = 1.0

            # Status control
            print("Robot in normal")
            print("Times: " + str(normal_times))
            print("Counter: " + str(self.normal_counter))
            print('Coords: ' + str(x) + ', ' + str(y))
            print('--------------------------')

	    #Go to the generated coords
	    self.act_c.send_goal(self.coords)
	    print('Goal: ' + str(x) + ', ' + str(y))

	    # Waits for the server to finish performing the action.
	    self.act_c.wait_for_result()

            while not rospy.is_shutdown():
		# Check if there was a play command in between random walks
                if sm_command == "play":
		    print(sm_command)
                    sm_command = None
                    return 'play'

		#Let know to the command_recog node we can accept inputs
	    	self.pub.publish('normal')

                # If not, continue with the behavior
                if(self.normal_counter < normal_times):
		    
		    self.good_coord = False
                    while(self.good_coord == False):
            		x = random.randrange(-6,6)
            		y = random.randrange(-8,8)
	    		z = 0
		
			#Check if the random coordinates are reachable in the map
			if x >= 1 and y >= 4:
				self.good_coord = False
			elif x <= -1 and y <= -6:
				self.good_coord = False
			elif x >= 1 and x <=2 and y >= 1 and y <= 3:
				self.good_coord = False
			else:
				self.good_coord = True

                    normal_coord = Point(x = x, y = y, z = z)
                    
		    self.coords.target_pose.pose.position.x = normal_coord.x
	    	    self.coords.target_pose.pose.position.y = normal_coord.y
	    	    self.coords.target_pose.pose.position.z = normal_coord.z

	    	    self.coords.target_pose.header.frame_id = "map"
    	    	    self.coords.target_pose.pose.orientation.w = 1.0

                    self.normal_counter = self.normal_counter + 1

                    # Status control
                    print("Times: " + str(normal_times))
                    print("Counter: " + str(self.normal_counter))
                    print('Coords: ' + str(x) + ', ' + str(y))
                    print('--------------------------')

		    #Go to the generated coords
	    	    self.act_c.send_goal(self.coords)
	    	    print('Goal: ' + str(x) + ', ' + str(y))

	    	    # Waits for the server to finish performing the action.
	    	    self.act_c.wait_for_result()

                else: return 'sleep'



# define state SLEEP
class Sleep(smach.State):
    """ Class for the SLEEP state

    In Sleep state the dog goes to the home location and sleeps.This Class initiates and sets the home position and publishes the position to the node 

    State Machine:
    	SLEEP to NORMAL 

    Parameters:
    	sleep_x: (double) Sleep x coordinate 

    	sleep_y: (double) Sleep y coordinate

    	time_sleep: (int) Sleeping time (1,10)

    Attributes:
    	coords: (MoveBaseGoal())

    Actions:
    	act_c: Client for action /move_base
		calls the action to move the robot to the specified coordinates

		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'])      

	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

	#Initializations
	self.coords = MoveBaseGoal()  

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state SLEEP')

        # Coordinates of the sleep position
        sleep_x = rospy.get_param('sleep_x')
        sleep_y = rospy.get_param('sleep_y')
	sleep_z = 0
	
	sleep_coord = Point(x = sleep_x, y = sleep_y, z = sleep_z)

	self.coords.target_pose.pose.position.x = sleep_coord.x
	self.coords.target_pose.pose.position.y = sleep_coord.y
	self.coords.target_pose.pose.position.z = sleep_coord.z

	self.coords.target_pose.header.frame_id = "map"
    	self.coords.target_pose.pose.orientation.w = 1.0

        # Go to sleep position
	print("going to sleep")
	self.act_c.send_goal(self.coords)
	print('Sleep: ' + str(self.coords.target_pose.pose.position.x) + ', ' + str(self.coords.target_pose.pose.position.y))

	# Waits for the server to finish performing the action.
	self.act_c.wait_for_result()

	time_sleep = rospy.get_param('~time_sleep', 10)

        while not rospy.is_shutdown():

            # When arrived, sleep for a fixed time and then continue to NORMAL state
            print("Robot arrived to sleep")
            time.sleep(time_sleep)
            print("Robot woken")
            return 'wait'
        
# Callback functions
sm_command = None
sm_flag = None
sm_point = Point()
def cb_command(data):
    """ callback to get the command received on the terminal
    """
    global sm_command
    sm_command = data.data


def cb_flag(data):
    """ callback to set the arrived flag
    """
    global sm_flag
    sm_flag = data.data

def cb_point(data):
    global sm_point
    sm_point.x = data.x
    sm_point.y = data.y


# main
def main():
    """ State machine initialization

    Creates the state machine, add states and link their outputs.
    
    Transitions:

    	SLEEP to NORMAL


    """
    global sm_command, sm_flag

    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['COORDS'])


    rate = rospy.Rate(10) # 10hz
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={ 'sleep':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wait':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop':'NORMAL'},


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    
    # Wait for ctrl-c to stop the application

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
