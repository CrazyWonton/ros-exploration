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

def update_laserScanData(data):
	laserData = data

def getDistanceToWall(laserScanData):
	#param the laser scan data
	#finds the distance to the nearest object if applicable and returns it
	return 0
	
def computeControlSignal(distanceToWall):
	#takes in the distance to the wall
	#returns a control signal
	def noObstacles():
		return 0
	def approachingObstacle():
		return 0
	def atObstacle():
		return 0

	return 0
	


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
