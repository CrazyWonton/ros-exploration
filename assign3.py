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
previousDistance = [0.0,0.0,0.0]
state = ['blocked','blocked']
startTime = 0.0

def update_laserScanData(data):
	#param the laser scan data
	#finds the distance to the nearest object if applicable and returns it
	#640 scans are produced in this case, so we take the middle portion
	#	which is basically directly in front of the robot
	global distance
	
	numberOfEntries = len(data.ranges)
	dataRange = numberOfEntries/100
	lowerBound = (numberOfEntries/2) - dataRange
	upperBound = (numberOfEntries/2) + dataRange
	
	#compute distance on left
	total = 0
	numEntries = 0
	i = 0
	while i < (lowerBound - dataRange):
		if data.ranges[i] > 0.45 and data.ranges[i] < 10.0:
			total += data.ranges[i]
			numEntries+=1
		i+=1
	if numEntries > 0:
		distance[0] = (total/numEntries)
	else:
		distance[0] = 5
	
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
		distance[1] = 5

	#compute distance on right
	total = 0
	numEntries = 0
	i = upperBound
	while i > (upperBound + dataRange):
		if data.ranges[i] > 0.45 and data.ranges[i] < 10.0:
			total += data.ranges[i]
			numEntries+=1
		i+=1
	if numEntries > 0:
		distance[2] = (total/numEntries)
	else:
		distance[2] = 5

def computeControlSignal():
	#takes in the distance to the wall
	#returns a control signal
	global distance
	global state
	global previousDistance
	global startTime
	
	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	
	turnSpeed = 0.6
	velocity = 0.7
	
	#periodic stop and backup
	if (rospy.get_time()-startTime) > 5 and state[0] == 'coast' and state[1] == 'coast':
		startTime = rospy.get_time()
		vel_msg.linear.x = -velocity
		return vel_msg
		
	#coasting
	if distance[1] > 1.5:
		if state[0] == 'coast':
			if state[1] == 'coast':
				vel_msg.linear.x = velocity
			elif distance[0] > .75:
				state[1] = 'coast'
		elif distance[0] > .75:
			state[0] = 'coast'
	#turning
	else:
		if state[0] == 'blocked':
			if state[1] == 'blocked':
				vel_msg.angular.z = -turnSpeed
			else:
				state[1] = 'blocked'
		else:
			state[0] = 'blocked'
			
	#get stuck, no forward progress
	diffCenter = ((previousDistance[1]-distance[1])**2)
	diffLeft = ((previousDistance[0]-distance[0])**2)
	diffRight = ((previousDistance[2]-distance[2])**2)
	
	# or (diffLeft > 0 and diffLeft < 0.0025) or (diffRight > 0 and diffRight < 0.0025)
	#if  (diffCenter > 0 and diffCenter < 0.00125):
	#	state[0] = state[1] = 'blocked'
	#	vel_msg.angular.z = -turnSpeed
	
	previousDistance[0] = distance[0]
	previousDistance[1] = distance[1]
	previousDistance[2] = distance[2]

	return vel_msg

if __name__ == '__main__':
	rospy.init_node('assign3')

	#initialize laserData
	laserData = LaserScan()
	global currentVel
	global startTime
	
	#set subscribers
	rospy.Subscriber("/scan", LaserScan, update_laserScanData)
	
	#set publishers
	pubControlSignals = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=2)

	while not rospy.is_shutdown():
		rospy.sleep(0.3)

		robot_control_signal = computeControlSignal()
		pubControlSignals.publish(robot_control_signal)
