#!/usr/bin/env python

# import relevant libraries
import roslib
import rospy
import math
import tf

# The velocity command message
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

goal_x = None
goal_y = None
pos_x = None
pos_y = None
current_angle = None
new_waypoint = False

def get_waypoint_goal(msg):
	'''
	Gets the goal from A*
	'''
	global goal_x, goal_y, new_waypoint
	print "[move_in_square] triggered get_waypoint_goal"
	goal_x, goal_y = msg.x, msg.y
	new_waypoint = True

def get_pos_callback(odom_msg):
	'''
	sets the current x, y from odom
	'''
	#print "[move_in_square] triggered get_pos_callback"
	global pos_x, pos_y, current_angle
	pos_x = odom_msg.pose.pose.position.x
	pos_y = odom_msg.pose.pose.position.y
	quaternion = (
			odom_msg.pose.pose.orientation.x,
			odom_msg.pose.pose.orientation.y,
			odom_msg.pose.pose.orientation.z,
			odom_msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	current_angle = yaw

def cmdvel_callback(msg):
	# Check robot has stopped
	global new_waypoint
	print "[move_in_square] triggered cmdvel_callback"
	ask_pub.publish(True)
	if new_waypoint == False:
		return

	if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
		# Make a new Twist waypoint message
		waypoint = Twist()

		# Command a waypoint 20 units to the right of the current robot position
		print "goal/pos", goal_x, goal_y
		print "goal/pos", pos_x, pos_y
		waypoint.linear.x =  goal_y - pos_y
		waypoint.linear.y =  pos_x - goal_x
		waypoint.linear.z = 0.0

		# Command the robot to turn 90 degrees clockwise
		waypoint.angular.x = 0.0
		waypoint.angular.y = 0.0
		#angle = math.atan2((goal_y - pos_y), (goal_x - pos_x))
		#waypoint.angular.z =  angle - current_angle 
		waypoint.angular.z =  0 

		pub.publish(waypoint)
		ask_pub.publish(True)
		new_waypoint = False


if __name__ == "__main__":
	# Initialise the mode
	rospy.init_node('move_in_square')

	# Publish waypoint data to robot
	pub = rospy.Publisher('/base_link_goal',Twist,queue_size=10)
	ask_pub = rospy.Publisher('/ask_waypoint',Bool,queue_size=10)
	sub = rospy.Subscriber('/cmd_vel',Twist,cmdvel_callback)
	goal_sub = rospy.Subscriber('/waypoints',Point,get_waypoint_goal)
	pos_sub = rospy.Subscriber('/odom',Odometry,get_pos_callback)

	ask_pub.publish(True)
	# Turn control over to ROS

	rospy.spin()
