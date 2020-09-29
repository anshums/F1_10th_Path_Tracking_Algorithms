#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global Variables
xc = 0
yc = 0
yaw = 0 
distance_front = []
min_front_dist = 0
radius_of_bubble  = 3

def pose_callback(data):	
	global xc,yc,yaw
	xc = data.pose.pose.position.x
	yc = data.pose.pose.position.y
	# getting yaw
	qx=data.pose.pose.orientation.x
	qy=data.pose.pose.orientation.y
	qz=data.pose.pose.orientation.z
	qw=data.pose.pose.orientation.w
	quaternion = (qx,qy,qz,qw)
	euler = euler_from_quaternion(quaternion)
	yaw = euler[2]
	# print(xc, yc, yaw)

def calcualte_coordinates(idx, distance_front):
	angle = (360 + idx)/4
	theta_0 = yaw + 135 - angle
	x = xc + distance_front[idx]*np.cos(np.deg2rad(theta_0))
	y = yc + distance_front[idx]*np.sin(np.deg2rad(theta_0))
	return x,y

def find_distance(x1,y1, x2,y2):
	distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
	return distance

def call_back(msg):
	global distance_front
	global min_front_dist
	distance_front = []
	i = 0
	while (i < len(msg.ranges)):
		if (i >= 360 and i <= 720 ):
			data = msg.ranges[i]
			if (msg.ranges[i] >= 3):
				data = 3.0
			distance_front.append(round(data,2))
		i = i+1
	# Finding Minimum distance in front
	min_front_dist = min(distance_front)
	idx = distance_front.index(min_front_dist)	# index of minimum front index
	min_idx = idx
	left_idx = idx
	min_x, min_y = calcualte_coordinates(min_idx, distance_front)
	distance = 0
	while (distance < radius_of_bubble and idx < len(distance_front)): 
		new_x, new_y = calcualte_coordinates(idx, distance_front)
		distance = find_distance (min_x, min_y, new_x, new_y)
		distance_front[idx] = 0
		idx += 1
	idx = left_idx
	while (distance < radius_of_bubble and idx > 0):
		new_x, new_y = calcualte_coordinates(idx, distance_front)
		distance = find_distance (min_x, min_y, new_x, new_y)
		distance_front[idx] = 0
		idx -= 1
	
	# print(len(distance_front))



def follow_the_gap(list):
	# rospy.sleep(0.1)
	list.reverse()
	distance_list = []
	# print('here')
	if len(list) == 0:
		return
	# if (distance_front)
	if (list[0] == 0 or list[len(list)-1] == 0):
		for dis in list:
			if (dis != 0):
				distance_list.append(dis)
	else:
		first_list = []
		second_list = []
		# i = 0
		flag = 0
		for i in list:
			# print(len(distance_front))
			# print("i: ", i)
			if (i == 0):
				flag = 1
			if (flag == 0 and i != 0):
				first_list.append(i)
			if (flag == 1 and i != 0):
				second_list.append(i)
		
		if (len(first_list) >= len(second_list)):
			distance_list = first_list
		else:
			distance_list = second_list
	
	# far_point = 
	if (len(distance_list) == 0):
		return
	far_point = max(distance_list)
	# print("far-point-: ", far_point,'\n')
	# print(list)
	#  distance_front.index(far_point))
	# print(distance_front.index(far_point))
	far_idx = list.index(far_point)
	
	steer = (180 - far_idx)/4
	vel_msg.drive.speed = 1
	p = 0.005
	vel_msg.drive.steering_angle = p*steer
	print("steer: ", vel_msg.drive.steering_angle)
	vel_pub.publish(vel_msg)

if __name__ == '__main__':
	rospy.init_node('follow_the_gap')
	vel_pub = rospy.Publisher('ego_id/drive', AckermannDriveStamped, queue_size=10)
	rospy.Subscriber("/ego_id/odom", Odometry, pose_callback)
	rospy.Subscriber('ego_id/scan', LaserScan, call_back)
	vel_msg = AckermannDriveStamped()
	
	while not rospy.is_shutdown():
		# rospy.sleep(2)
		follow_the_gap(distance_front)
		rate = rospy.Rate(10)
		rate.sleep()