#!/usr/bin/env python

"""
VoiceTeleop.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('GuideBot')
import rospy
import math
import time
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import String



class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)   
	self.paused = False  
        #Default Speed	
        self.speed = 0.2
	self.msg = Twist()

        # publish to cmd_vel, subscribe to speech output
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            r.sleep()
    #speech callback function    
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
	
        # halts speech when user says pause speech 
	if msg.data.find("pause speech") > -1:
            self.paused = True
            os.system("espeak -v en \"speech paused\"")
        #speech commannd resumes when user says continue speech	       
        elif msg.data.find("continue speech") > -1:
            self.paused = False
            os.system("espeak -v en \"speech resumed\"") 
        if self.paused:
           return   
        # Robot responds to the phrases below using espeak(text to speech)
        if msg.data.find("how are you") > -1: 
            os.system("espeak -v en \"i am good and you\"" ) 
        elif msg.data.find("how old are you") > -1: 
            os.system("espeak -v en \"i am still a baby\"" ) 
        elif msg.data.find("you look cool") > -1: 
            os.system("espeak -v en \"thanks\"" ) 
        elif msg.data.find("you are smart") > -1: 
            os.system("espeak -v en \"really you are such an honest person\"" )  
        elif msg.data.find("i like you") > -1: 
            os.system("espeak -v en \"i like you too\"" )    
        elif msg.data.find("weather") > -1: 
            os.system("espeak -v en \"it is twenty degrees fahrenheit\"" ) 
        elif msg.data.find("time") > -1: 
            os.system("espeak -v en \"" + time.strftime("%H:%M") + "\"" ) 
        
        # Movement Commands

        #moves forward
        elif msg.data.find("move forward") > -1 or msg.data.find("go forward") > -1 :    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
	
        # rotates left      
	elif msg.data.find("rotate left") > -1 or msg.data.find("turn left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 0.6	
            else:        
                self.msg.angular.z = 0.8 #self.speed *2
	# rotates right
        elif msg.data.find("rotate right") > -1 or msg.data.find("turn right") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z = - 0.6
            else:        
                self.msg.angular.z = - 0.8 #self.speed *2
	# strafe left
	elif msg.data.find("move left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z = 0.6	
            else:        
                self.msg.angular.z = 0.8 #self.speed *2
	
	# strafe right	
	elif msg.data.find("move right") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z = -0.6	
            else:        
                self.msg.angular.z = -0.8 #self.speed *2

	# move backwards
        elif msg.data.find("move back") > -1 or msg.data.find("go back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0

	# move 50 cm approximately
	#elif msg.data.find("forward") > -1: 
	#    count = time.time()   
        #    self.msg.linear.x = self.speed
        #    self.msg.angular.z = 0
	#    self.pub.publish(self.msg)		
        #    while time.time() - count < 2.5:
	#		rospy.loginfo(time.time() - count)  
	#    self.msg = Twist()
	#    self.pub.publish(self.msg)		      
	
	# stops the robot	
	elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:          
            self.msg = Twist()
        
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
	    self.cleanup

    def cleanup(self):
        # stops the robot!
        twist = Twist()
        self.pub.publish(twist)


if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
    	voice_cmd_vel()
    	
    except:
        pass
	


    
