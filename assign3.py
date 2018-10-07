//The purpose of this program is to generate control signals based on laser scan readings
/**

Subscribe to the sensor_msgs/LaserScan or /scan topic; 
get like the middle 3 values and average them to get a general distance straight ahead; 
	if D >1, keep going straight at MAXV, 
	if D<.5, turn right 90degrees, \
	else reduce speed to MAXV/constant or MAXV*D;
publish control signal to cmd_vel.

**/

import rospy
import roslib
from geometry_msgs.msg import Twist

