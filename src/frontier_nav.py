#!/usr/bin/env python

occ_threshold = 80

# import relevant libraries
import roslib
import rospy
import math
import message_filters
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from math import sqrt

from std_msgs.msg import Bool
from geometry_msgs.msg import Point

Goal_Requested = True

def h_x(start, end):
	'''
	Calculates the L2 norm between "start" and "end" nodes.
	Note: The L2 norm is admissible in a 2D plane.
	'''
	return sqrt( (start[0] - end[0])**2 + (start[1]-end[1])**2)

def find_frontier(map2d, rob_x, rob_y):#, rows, cols):
	'''
	Finds the frontier of points that the robot should explore near.
	This is done by finding the points where there is an open space next to a
	unseen space. Returns the SEEN, OPEN spaces that satisfy this property.
	'''
	global occ_threshold
	print '[frontier_nav] finding waypoint...'
	frontier_point = None
	dist = None
	frontier_count = 0
	for row in range(len(map2d)):
		for col in range(len(map2d[0])):
			if map2d[row][col] != -1 and map2d[row][col] < occ_threshold:
				neighbors = []
				if row != 0:
					neighbors.append(map2d[row-1][col])
				if row != len(map2d)-1:
					neighbors.append(map2d[row+1][col])
				if col != 0:
					neighbors.append(map2d[row][col-1])
				if col != len(map2d[0])-1:
					neighbors.append(map2d[row][col+1])
				# Find if any of the neighbors of this node are unseen	
				for n in neighbors:
					# If so, append this node to the frontier
					if n == -1:
						frontier_count += 1

						if frontier_point is None:
							frontier_point = (row, col)
							dist = h_x((rob_x, rob_y), frontier_point)
							break;

						temp_dist = h_x( (rob_x, rob_y), (row, col))
						if ( temp_dist < dist):
							frontier_point = (row, col)
							dist = temp_dist
						break;
	print "[frontier_nav] Frontier_point: ", frontier_point, "count: ", frontier_count
	return frontier_point

def frontier_callback(map_msg, odom_msg):
	'''
	Uses the published map (/map) to see if the robot is in 
	an enclosed space.
	'''
	global Goal_Requested
	print "[frontier_nav] In callback"

	if Goal_Requested == False:
		print '[frontier_nav] GOAL NOT NEEDED. Skipping search'
		return

	print "[frontier_nav] width, height: ", map_msg.info.width, map_msg.info.height
	print "[frontier_nav] origin: ", map_msg.info.origin.position.x, map_msg.info.origin.position.y
	# Turn the map into a 2-d numpy array
	map2d = np.array(map_msg.data)#, (map_msg.info.width, map_msg.info.height))
	map2d = np.reshape(map2d, (map_msg.info.width, map_msg.info.height))
	#print map2d

	# Convert ODOM into map indicies
	pos_x = odom_msg.pose.pose.position.x
	pos_y = odom_msg.pose.pose.position.y
	grid_x = ((pos_x - map_msg.info.origin.position.x) / map_msg.info.resolution)
	grid_y = ((pos_y - map_msg.info.origin.position.x) / map_msg.info.resolution)

	frontier_point = find_frontier(map2d, grid_x, grid_y)#, map_msg.info.width, map_msg.info.height)
	print "[frontier_nav] CLOSEST POINT:", frontier_point

	# Publish the point on /frontier_goal
	print '[frontier_nav] PUBLISHING GOAL'
	p = Point()
	p.x = frontier_point[0]
	p.y = frontier_point[1]
	point_pub.publish(p)
	Goal_Requested = False


def goal_request_callback(msg):
	global Goal_Requested
	print "[frontier_nav] GOAL REQUESTED", msg
	if msg.data == True:
		Goal_Requested = True

if __name__ == "__main__":
	# Initialise the mode
	rospy.init_node('frontier_nav')
	map_sub = message_filters.Subscriber('/map', OccupancyGrid)
	odom_sub = message_filters.Subscriber('/odom', Odometry)


	# The filter joins the two subscribers with a message queue size of 10, and a .5 second "slop"
	ts = message_filters.ApproximateTimeSynchronizer([map_sub, odom_sub], 10, .5)
	ts.registerCallback(frontier_callback)

	# Publish the found node
	point_pub = rospy.Publisher('/frontier_goal', Point,queue_size=10)

	# Subscribe to goal_requests
	goal_sub = rospy.Subscriber('/frontier_goal_request', Bool, goal_request_callback)

	# Turn control over to ROS
	rospy.spin()
