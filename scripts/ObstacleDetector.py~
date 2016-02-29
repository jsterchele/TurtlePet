#!/usr/bin/env python

# Obstacle detection node
# Every python controller needs these lines
import roslib; roslib.load_manifest('GuideBot')
import rospy
import numpy
import math
import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import os

# The velocity command message
from geometry_msgs.msg import Twist

# The point cloud
from geometry_msgs.msg import Point

#from geometry_msgs.msg import Quaternion

# The laser scan message
from sensor_msgs.msg import LaserScan

# The quaternion message
from sensor_msgs.msg import Imu

class Demo:
    def __init__(self):
        
	
	#check if turtlbot has turned 90 degrees 
	self.hasTurned = False
        
	#check if turtlebot has spoken	
	self.hasSpoken = False
        # Subscriber for the laser data
	rospy.Subscriber('scan', LaserScan, self.laser_callback)
	# Subscriber for the orientation data     	
        #rospy.Subscriber('mobile_base/sensors/imu_data', Imu, self.gyro_callback)
	# Subscriber for voice commands
	rospy.Subscriber('/recognizer/output', String, self.lastCalled)
        # set initial orientation to zero	
        self.orientation = 0
	

        # A publisher for the move data
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist) # might delete this
        # An indirect publisher for move data through the voice command node	
        self.pub2 = rospy.Publisher('recognizer/output', String) 
	        
        # Let the world know we're ready
        rospy.loginfo('Stopper initialized') 
	

	
	
   # sets the check boolean val to true after the bot has turned 90 degrees.

    #def gyro_callback(self, gyroscan):
#	if self.hasTurned == False:
#		self.orientation = gyroscan.orientation.z
#	if math.fabs(gyroscan.orientation.z - self.orientation) > 0.64 #and self.hasTurned == True:
#		self.hasTurned = False
	
#	print(str(gyroscan.orientation.z))
#	print(self.hasTurned)
	
   # sets the check boolean val to false after 'move left','move right', ' rotate right', 'rotate left' and 'move back' commands are executed
    def lastCalled(self, msg):
	if msg.data.find("move left") > -1 or msg.data.find("move right") or msg.data.find("rotate left") > -1 or msg.data.find("rotate right")  > -1 or msg.data.find("move back")  > -1 and self.hasTurned == False:
		self.hasTurned = True
    
    #callback function for the laserscan         
    def laser_callback(self, scan):
       
	soundhandle = SoundClient()
	pointsX = []	
	pointsY = []
	angle = scan.angle_min
        # Gets rid of nan values and stores x and y coordinates of the scan in separate lists but with corresponding index	
        for rad in scan.ranges:
					
		if not numpy.isnan(rad):					
			pointsX.append(rad*math.cos(angle))
			pointsY.append(rad*math.fabs(math.sin(angle)))
		angle += scan.angle_increment
			
	command = Twist()
	
	
	if len(pointsX) != 0:
		
		# Sets the check to false
		if min(pointsX) > 0.9:
			self.hasTurned = False
			self.hasSpoken = False
                # Check if there is an obstacle 0.9 m in front of it and 0.2 m to its side		
                elif  min(pointsX) < 0.9 and math.fabs(pointsY[pointsX.index(min(pointsX))]) <= 0.2 and self.hasTurned == False:
			self.pub2.publish('stop')
                        firstIndex = -1
                        self.hasTurned = False
                        # stores the values of the first index with the given value for the x coordinates			
                        for i in pointsX:
				if i < 0.9 and firstIndex == -1:
					firstIndex = pointsX.index(i)
				if i < 0.9:
					lastIndex = pointsX.index(i)
			
			if math.fabs(pointsY[firstIndex]) < math.fabs(pointsY[lastIndex]):
				if not self.hasSpoken:
					# Command line expression for text to speech
                                        os.system("espeak -v en \"Obstacle detected. I recommend going " + str(int(100 * pointsY[lastIndex] + 20)) + " centimeters to the right to avoid the obstacle.\"")
                                       	self.hasSpoken = True	
							
			else:
				if not self.hasSpoken:

                                        os.system("espeak -v en \"Obstacle detected. I recommend going " + str(int(100 * pointsY[firstIndex] + 20)) + " centimeters to the left to avoid the obstacle.\"")
                                       	self.hasSpoken = True
		                                		#Wall Detector																						           
                elif min(pointsX) < 0.9 and math.fabs(pointsY[(len(pointsY)/2)]) <= 0.2 and self.hasTurned == False:
                        self.pub2.publish('stop')
                        if not self.hasSpoken:

                                        os.system("espeak -v en \"Wall Detected\"")
                                        self.hasSpoken = True
                #rospy.loginfo('min(Point X): {0}, Point Y:  {1}'.format(min(pointsX) , pointsY[pointsX.index(min(pointsX))] ))
																									      
	#rospy.loginfo('min(Point X): {0}, Point Y:  {1}'.format(min(pointsX) , pointsY[(len(pointsX)/2)] ))#pointsY[pointsX.index(min(pointsX))] ))      
	
if __name__ == '__main__':
    rospy.init_node('cmd_vel_mux')
    
    demo = Demo()
    	 
    rospy.spin()

