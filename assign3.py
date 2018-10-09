#! /usr/bin/env python

#The purpose of this program is to generate control signals based on laser scan readings
#
#
#Subscribe to the sensor_msgs/LaserScan or /scan topic; 
#get like the middle 3 values and average them to get a general distance straight ahead;
#	if D >1, keep going straight at MAXV, 
#	if D<.5, turn right 90degrees, \
#	else reduce speed to MAXV/constant or MAXV*D;
#publish control signal to cmd_vel.
#

import rospy
import roslib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random
import math

#left middle right
distance = [0.0,0.0,0.0]
state = 'stationary'

def update_laserScanData(data):
	#param the laser scan data
	#finds the distance to the nearest object if applicable and returns it
	#640 scans are produced in this case, so we take the middle portion
	#	which is basically directly in front of the robot
	global distance
	global state
	
	numberOfEntries = len(data.ranges)
	dataRange = numberOfEntries/50
	lowerBound = (numberOfEntries/2) - dataRange
	upperBound = (numberOfEntries/2) + dataRange
	
	#compute distance on left
	total = 0
	numEntries = 0
	i = 0
	while i < dataRange:
		if data.ranges[i] > 0.45 and data.ranges[i] < 10.0:
			total += data.ranges[i]
			numEntries+=1
		i+=1
	if numEntries > 0:
		distance[0] = (total/numEntries)
	else:
		distance[0] = 0
	
	#compute distance in center
	total = 0
	numEntries = 0
	i = lowerBound
	while i < upperBound:
		if data.ranges[i] > 0.45 and data.ranges[i] < 10.0:
			total += data.ranges[i]
			numEntries+=1
		i+=1
	
	if numEntries > 0:
		distance[1] = (total/numEntries)
	else:
		distance[1] = 10

	#compute distance on right
	total = 0
	numEntries = 0
	i = numberOfEntries - 1
	while i > (numberOfEntries-dataRange):
		if data.ranges[i] > 0.45 and data.ranges[i] < 10.0:
			total += data.ranges[i]
			numEntries+=1
		i-=1
	if numEntries > 0:
		distance[2] = (total/numEntries)
	else:
		distance[2] = 0

def computeControlSignal():
	#takes in the distance to the wall
	#returns a control signal
	global distance
	
	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	
	fastVel = .5
	slowVel = .2
	largeTurn = .5
	smallTurn = .1
	
	if distance[0] > 1.5:
		if distance[1] > 1.5:
			if distance[2] > 1.5: 			#coast 000
				vel_msg.linear.x = fastVel
				print 'coast 000'
			else:							#slight left 001
				vel_msg.linear.x = slowVel
				vel_msg.angular.z = smallTurn
				print 'slight left 001'
		else:
			if distance[2] > 1.5: 			#turn right 010
				vel_msg.angular.z = -largeTurn
				print 'turn right 010'
			else:							#turn left 011
				vel_msg.angular.z = -largeTurn
				print 'turn left 011'
	else:
		if distance[1] > 1.5:
			if distance[2] > 1.5: 			#slight right 100
				vel_msg.linear.x = slowVel
				vel_msg.angular.z = -smallTurn
				print 'slight right 100'
			else:							#approach slowly 101
				vel_msg.linear.x = slowVel
				print 'approach slowly 101'
		else:
			if distance[2] > 1.5: 			#turn right 110
				vel_msg.angular.z = -largeTurn
				print 'turn right 110'
			else:							#turn right 111
				vel_msg.angular.z = -largeTurn
				print 'turn right 111'

	return vel_msg

if __name__ == '__main__':
	rospy.init_node('assign3')

	#initialize laserData somehow
	laserData = LaserScan()
	global currentVel
	
	#set subscribers
	rospy.Subscriber("/scan", LaserScan, update_laserScanData)
	
	#set publishers
	pubControlSignals = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=2)

	startTime = rospy.get_time()
	while not rospy.is_shutdown():
		rospy.sleep(0.4)

		robot_control_signal = computeControlSignal()
		pubControlSignals.publish(robot_control_signal)
