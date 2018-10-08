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

global laserData
global distance
global currentVel

def update_laserScanData(data):
	laserData = data

def getDistanceToWall(ld):
	#param the laser scan data
	#finds the distance to the nearest object if applicable and returns it
	#640 scans are produced in this case, so we take the middle portion
	#	which is basically directly in front of the robot
	i = 315
	total = 0
	numEntries = 0
	while i < 325:
		if ld[i] > 0.45 && ld[i] < 10.0:
			total += ld[i]
			numEntries++

	return (total/numEntries)
	
def computeControlSignal(d):
	#takes in the distance to the wall
	#returns a control signal
	def noObstacles():
		vel_msg = Twist()
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		
		if currentVel < 1:
			vel_msg.linear.x = currentVel + 0.1
		else
			vel_msg.linear.x = currentVel
		
		currentVel = vel_msg.linear.x
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		return vel_msg
		
	def approachingObstacle():
		vel_msg = Twist()
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		
		if currentVel > 0.5:
			vel_msg.linear.x = currentVel - 0.1
		else
			vel_msg.linear.x = currentVel
		
		currentVel = vel_msg.linear.x
		vel_msg.angular.x
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		return vel_msg

	def atObstacle():
		vel_msg = Twist()
		vel_msg.linear.x = 0
		currentVel = vel_msg.linear.x
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0.5
		return vel_msg


	if d > 3:
		return noObstacles()
	elif d > 1:
		return approachingObstacle()
	else
		return atObstacle()
	


if __name__ == '__main__':
	rospy.init_node('assign3')

	#initialize laserData somehow

	#set subscribers
	rospy.Subscriber("/scan", sensor_msgs/LaserScan, update_laserScanData)
	
	#set publishers
	pubControlSignals = rospy.Publisher('/predator/cmd_vel', Twist, queue_size=2)

	startTime = rospy.get_time()
	while not rospy.is_shutdown():
		rospy.sleep(0.1)

		robot_control_signal = computeControlSignal(getDistanceToWall(laserData))
		pubControlSignals.publish(robot_control_signal)
