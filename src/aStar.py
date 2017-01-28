#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import message_filters
import numpy as np

# import messages
from std_msgs.msg import Float32MultiArray, Bool, Header
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry

SCAN_RESOLUTION = 2
#SCAN_RANGE = [[-SCAN_RESOLUTION,0],[SCAN_RESOLUTION,0],[0,-SCAN_RESOLUTION],[0,SCAN_RESOLUTION]]

SCAN_RANGE = [[-SCAN_RESOLUTION,-SCAN_RESOLUTION],[-SCAN_RESOLUTION,0],[-SCAN_RESOLUTION,SCAN_RESOLUTION]			# sensor will only scan the neibouring 8 cells aroung it
				,[0,-SCAN_RESOLUTION],[0,SCAN_RESOLUTION]  
				,[SCAN_RESOLUTION,-SCAN_RESOLUTION],[SCAN_RESOLUTION,0],[SCAN_RESOLUTION,SCAN_RESOLUTION]]



# Global vars
START = [0,0]	# odom_scale
WAYPOINT_REQUEST_FLAG = None
NEXT_WAYPOINT = Point()
NEXT_WAYPOINT.x = 0
NEXT_WAYPOINT.y = 0

map2d = None
odom_X = None
odom_Y = None
path = []

class Node:
	def __init__(self, pos=[]):
		self.pos = pos
		self.row = pos[0]
		self.col = pos[1]
		self.g_cost = None
		self.h_cost = None
		self.parent = None
		#self.total_cost = self.g_cost + self.h_cost

def get_Euclidean_dist(pos1, pos2):
	cost = abs(pos1[0]-pos2[0])**2 + abs(pos1[1]-pos2[1])**2
	return math.floor(math.sqrt(cost))


def aStarAlog(Map, Start, Goal):
	#print Map
	print '[aStar] in A star function'
	#print start
	#print goal
	map_width = len(Map)
	start_node = Node(Start)
	start_node.g_cost = 0
	start_node.h_cost = 0

	open_list = set()	# store all possible next parent nodes
	close_list = []	# store all parent nodes
	open_list.add(start_node)

	while open_list:

		cur_node = min(open_list, key=lambda item:item.g_cost + item.h_cost)	# get least value in the open list
		
		#if cur_node.pos == Goal:	# check if the current pos is the goal
		if (abs(cur_node.row - Goal[0]) <= SCAN_RESOLUTION) and (abs(cur_node.col - Goal[1]) <= SCAN_RESOLUTION):
			path = []
			while cur_node.parent:
				path.append(cur_node)
				cur_node = cur_node.parent
			path.append(cur_node)
			return path

		close_list.append(cur_node.pos)	# rememeber visted location
		open_list.remove(cur_node)

		for i in range(len(SCAN_RANGE)):
			if SCAN_RANGE[i][0] == 0 or SCAN_RANGE[i][1] == 0:	# not diagonal position
				g_cost_value = 1
			else:
				g_cost_value = 1.4	# diagonal position

			scanned_cell_pos = [sum(x) for x in zip(SCAN_RANGE[i], cur_node.pos)]
			scanned_cell_row, scanned_cell_col = scanned_cell_pos[0], scanned_cell_pos[1]
			scanned_node = Node(scanned_cell_pos)	# scan thoughout four directions

			if scanned_node.pos not in close_list:	
				if (( -1 < scanned_cell_row < map_width) and ( -1 < scanned_cell_col < map_width) # check not out of range
					and (Map[scanned_cell_row][scanned_cell_col] == 0)):	# not occupant
					# process for valid cell:

					in_openList = False 	# flagged if scanned cell is in the open list already (0 assume it's not)
					for item in open_list:
						if scanned_cell_pos == item.pos:	# found repeated scanned node
							in_openList = True
							new_g_cost = g_cost_value + cur_node.g_cost
							if new_g_cost < item.g_cost:
								item.g_cost = new_g_cost	# update g cost is is less than what it was
								item.parent = cur_node		# update parent

					if in_openList == False:
						scanned_node.g_cost = g_cost_value + cur_node.g_cost
						scanned_node.h_cost = get_Euclidean_dist(scanned_cell_pos, Goal)
						total_cost = scanned_node.g_cost + scanned_node.h_cost
						scanned_node.parent = cur_node
						open_list.add(scanned_node)


#===========================================
#			callback functions
#===========================================

def get_waypoint_callback(msg):		# listen to "/frontier_goal"	
  global NEXT_WAYPOINT
  print '[astar] get_waypoint_callback triggered'
  NEXT_WAYPOINT = msg


def map_odom_callback(map_msg, odom_msg):
    '''
    records the map and odom data in the global scope
    '''
    global map2d, odom_X, odom_Y, origin_X, origin_Y, resolution
    map2d = np.array(map_msg.data)
    map2d = np.reshape(map2d, (map_msg.info.width, map_msg.info.height))
    odom_X = odom_msg.pose.pose.position.x 
    odom_Y = odom_msg.pose.pose.position.y 
    origin_X = map_msg.info.origin.position.x 	# -100 	
    origin_Y = map_msg.info.origin.position.y 	# -100
    resolution = map_msg.info.resolution

def ask_waypoint_callback(msg):
    global path
    if len(path) == 0:
    	print "[aStar]: Path is empty, asking new waypoint..."
        pub2.publish(True)
    else:
        p = Point()
        next_pos = path[0]	# this is in the grid scale
        del path[0]
        p.x = next_pos[0]#int(next_pos[0] * resolution + origin_X)
        p.y = next_pos[1]#int(next_pos[1] * resolution + origin_Y)
        #print "[aStar]: Publishing next goal point in the path: [%d,%d]" % (p.x,p.y)
        pub.publish(p)	# Point type: first position of the path


def frontier_goal(goal_p):
	global map2d, odom_X, odom_Y, path, origin_X, origin_Y, resolution
	print "[aStar]: Current Goal: [%d,%d]" % (int(goal_p.x * resolution + origin_X), int(goal_p.y * resolution + origin_Y))
	currentX = (odom_X - origin_X) / resolution	# 0 -> 1999
	currentY = (odom_Y - origin_Y) / resolution	# 0 -> 1999
	current_pos = [int(currentX),int(currentY)]

	result = aStarAlog(map2d, current_pos, [goal_p.x, goal_p.y])
	path = []
	for item in result:		# store the path as a list
		odom_scale_x = int(item.row * resolution + origin_X)
		odom_scale_y = int(item.col * resolution + origin_Y)
		path.append([odom_scale_x,odom_scale_y])
	ask_waypoint_callback(None)


if __name__ == "__main__":

	#global pub
	rospy.init_node("aStarPath")

	map_sub = message_filters.Subscriber('/map', OccupancyGrid)
	odom_sub = message_filters.Subscriber('/odom', Odometry)
	waypoint_bool_sub = rospy.Subscriber('/ask_waypoint', Bool, ask_waypoint_callback)


	waypoint_sub = rospy.Subscriber('/frontier_goal', Point, frontier_goal)
	ts = message_filters.TimeSynchronizer([map_sub, odom_sub], 10)

	pub = rospy.Publisher('/waypoints',Point)
	pub2 = rospy.Publisher('/frontier_goal_request',Bool)
	pub2.publish(True)
	ts.registerCallback(map_odom_callback)

	rospy.spin()
