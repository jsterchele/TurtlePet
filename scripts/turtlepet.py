#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.
import roslib; roslib.load_manifest('turtlepet')
import rospy
import numpy as np
import os
import math import tanh
import random
import time import sleep

# The laser scan message
from sensor_msgs.msg import LaserScan
# The geometry movement message
from geometry_msgs.msg import Twist
# The standard string message
from std_msgs.msg import String
# The sound_play message
from sound_play.libsoundplay import SoundClient
# The state machine
from smach import State,StateMachine

# This class will follow the nearest thing to it.
class follower(State):
    def __init__(self, followDistance, stopDistance, max_speed, min_speed=0.01 ):
        # Subscribe to the laser data
        self.sub = rospy.Subscriber('scan', LaserScan, self.execute)
        self.subz = rospy.Subscriber('recognizer/output', String, self.execute)

        # Publish movement commands to the turtlebot's base
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)

        # How close should we get to things, and what are our speeds?
        self.stopDistance = stopDistance
        self.max_speed = max_speed
        self.min_speed = min_speed
	#at what distance do we start following something/someone?
	self.followDist = followDistance

	#the distance to the closest object, and its position in array, respectively.
	self.closest = 0
	self.position = 0
        # Create a Twist message, and fill in the fields.  We're also
        # going to fill in the linear.x field, even though we think
        # we're going to set it later, just to be sure we know its
        # value.
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

    def execute(self, scan, recognizer):
	#determines the closest thing to the Robit.	
	self.getPosition(scan)
	rospy.logdebug('position: {0}'.format(self.position))
	
	#if there's something within self.followDist from us, start following.
	if (self.closest < self.followDist):
	    self.follow()	
	#else just don't run at all.	
	else:
	    self.stop() 

        # Add a log message, so that we know what's going on
        rospy.logdebug('Distance: {0}, speed: {1}, angular: {2}'.format(self.closest, self.command.linear.x, self.command.angular.z))
	#Ensure we have only one publish command.
	self.pub.publish(self.command)
        if msg.data.find("wander") > -1:
	    return 'wander'

        elif msg.data.find("follow") > -1:
	    return 'follow'

	#Starts following the nearest object.
    def follow(self):
	self.command.linear.x = tanh(5 * (self.closest - self.stopDistance)) * self.max_speed
	#turn faster the further we're turned from our intended object.
	self.command.angular.z = ((self.position-320.0)/320.0)
	
	#if we're going slower than our min_speed, just stop.
	if abs(self.command.linear.x) < self.min_speed:
    	self.command.linear.x = 0.0  

    def stop(self):
	self.command.linear.x = 0.0
	self.command.angular.z = 0.0
	#function to occupy self.closest and self.position

    def getPosition(self, scan):
        # Build a depths array to rid ourselves of any nan data inherent in scan.ranges.
	depths = []
	for dist in scan.ranges:
	    if not np.isnan(dist):
			depths.append(dist)
	#scan.ranges is a tuple, and we want an array.
	fullDepthsArray = scan.ranges[:]

	#If depths is empty that means we're way too close to an object to get a reading.
	#thus establish our distance/position to nearest object as "0".
	if len(depths) == 0:
	    self.closest = 0
	    self.position = 0
	else:
	    self.closest = min(depths)
	    self.position = fullDepthsArray.index(self.closest)


class listen(State):

    #init function
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
	self.paused = False
        #Default Speed
        self.speed = 0.25
	self.msg = Twist()
        
        self.sound_client = SoundClient()        
        rospy.sleep(2)        

        # publish to cmd_vel, subscribe to speech output
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        rospy.Subscriber('recognizer/output', String, self.execute)
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            r.sleep()
        rospy.init_node('play_sound_file')



    #speech callback function    
    def execute(self, msg):
        rospy.loginfo(msg.data)
	# spin command
        if msg.data.find("spin left") > -1 :
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 2.3	
            else:        
                self.msg.angular.z = 2.3

        elif msg.data.find("spin right") > -1 :
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = - 2.3	
            else:        
                self.msg.angular.z = - 2.3

        elif msg.data.find("spin") > -1 :
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 2.3	
            else:        
                self.msg.angular.z = 2.3

        elif msg.data.find("speak") > -1 : 
            self.sound_client.playWave('/opt/ros/indigo/share/sound_play/sounds/BACKINGUP.ogg')

        elif msg.data.find("sing") > -1 : 
            c = random.choice(seq)
            self.sound_client.playWave(c)


	#These following commmands are movement commands.
	#moves forward
        elif msg.data.find("move forward") > -1 or msg.data.find("go forward") > -1 :    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
	
        # rotates to the left      
	elif msg.data.find("rotate left") > -1 or msg.data.find("turn left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 0.6	
            else:        
                self.msg.angular.z = 0.8 
	# rotates to the right
        elif msg.data.find("rotate right") > -1 or msg.data.find("turn right") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z = - 0.6
            else:        
                self.msg.angular.z = - 0.8 
	# Moves to the left
	elif msg.data.find("move left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 0.6	
            else:        
                self.msg.angular.z = 0.8 
	
	# Moves to the right	
	elif msg.data.find("move right") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z = -0.6	
            else:        
                self.msg.angular.z = -0.8 

	# Moves backwards
        elif msg.data.find("move back") > -1 or msg.data.find("go back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0

        # Stops the robot	
	elif msg.data.find("stop") > -1:          
            self.msg = Twist()

	elif msg.data.find("wander") > -1:
	    return 'wander'

        elif msg.data.find("follow") > -1:
	    return 'follow'

        # strafe left or right- uses while loop for time lag 	
        if self.msg.angular.z != 0 and (msg.data.find("move left") > -1 or msg.data.find("move right") > -1):
                #turn 90 degrees to the left or right
				
                ang_vel = self.msg.angular.z
                # rotate		
                self.pub.publish(self.msg)
                count = time.time()
                while time.time() - count < 3.9:
                        rospy.loginfo(time.time() - count)
		
                # stop 	
                self.msg = Twist()	
                self.pub.publish(self.msg)
		
                # move forward		
                count = time.time()
                self.msg.linear.x = self.speed
                self.pub.publish(self.msg)
                while time.time() - count < 2.4:
                        rospy.loginfo(time.time() - count)
		
                # stop message	
                self.msg = Twist()	
                self.pub.publish(self.msg)


                #rotate 90 degrees to in the opposite direction of the previous turn
                self.msg.angular.z = -ang_vel
                self.pub.publish(self.msg)
                count = time.time()
                while time.time() - count < 3.9:
                        rospy.loginfo(time.time() - count)
                # stop message	
                self.msg = Twist()	
                self.pub.publish(self.msg)

           # rospy.sleep(math.pi/(2*self.msg.angular.z))
		
        # rotate right or left	
        elif self.msg.angular.z != 0:
                #turn 90 degrees to the left or right
                count = time.time()		
                self.pub.publish(self.msg)
                while time.time() - count < 3.9:
                        rospy.loginfo(time.time() - count)
		
                # stop message	
                self.msg = Twist()	
                self.pub.publish(self.msg)
			
        else:
            self.pub.publish(self.msg)	
            # Stops the robot from moving
            twist = Twist()
            self.pub.publish(twist)
        
def turtlepet():
    turtlepet = StateMachine(outcomes=['success'])
    rospy.Subscriber('recognizer/output', String, self.speechCb)
    

    #speech callback function    
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        rospy.loginfo("Im in a statemachine")
	# Listen to a command to put it into a new state.
        #if msg.data.find("wander") > -1 :
        #    StateMachine.add("listen", voice_cmd_vel(State), transitions={'follow': 'FOLLOW', 'listen': 'LISTEN'}) 

        #elif msg.data.find("turtle") > -1 or msg.data.find("pet") > -1 :
        #    StateMachine.add("listen", voice_cmd_vel(State), transitions={'follow': 'FOLLOW', 'wander': 'WANDER'})    

        #elif msg.data.find("follow") > -1 :
        #    StateMachine.add("follow", voice_cmd_vel(State), transitions={'listen': 'LISTEN', 'wander': 'WANDER'})  
        #return turtlepet



if __name__ == '__main__':
    tp = turtlepet()
    tp.execute()
	
