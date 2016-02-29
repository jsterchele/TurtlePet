#!/usr/bin/env python

import random
import rospy
from sensor_msgs.msg import LaserScan
import math

# Every Python Controller needs these lines
import roslib; roslib.load_manifest('turtlepet')

# The velocity command message
from geometry_msgs.msg import Twist


def callback(data):
    distance = .7
    # If a point in the range is less than a distance determined in the cmd line input, stop moving forward and turn until you can resume forward motion - at which point stop turning.
    midIndex = len(data.ranges)/2
    topIndex = int(math.ceil(midIndex + midIndex * .2))
    btmIndex = int(math.floor(midIndex - midIndex * .2))
    data.ranges = data.ranges[btmIndex:topIndex]
    no_nans = [n for n in data.ranges if not math.isnan(n)]
    if min(no_nans) < distance:
        rando = random.randint(0,9)
	if rando < 1:
            command.linear.x = 0
            command.angular.z = .5
        else:
            command.angular.z = -.5
    else:
        command.linear.x = 0.2
        command.angular.z = 0
   

# Main function
if __name__ == '__main__':
   
    rospy.init_node('move')

    # A publisher for the move data
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('scan', LaserScan, callback)

    # Drive forward at a given speed.  The robot points up the x-axis.
    command = Twist()
    command.linear.x = .3
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # Loop at 10Hz, publishing movement commands until we shut down.
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(command)
        rate.sleep()
