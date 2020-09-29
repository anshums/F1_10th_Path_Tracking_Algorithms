#!/usr/bin/env python
# Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg
from rospkg import RosPack
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import matplotlib.pyplot as plt
from matplotlib import patches

# GLOBAL VARIABLES
xc = 0
yc = 0
yaw = 0
idx = 0
wb = 0.3302
waypoints = []
stanley = True
show_animation = True
VEL_LOOKAHEAD_IDX = 8
MAX_VEL = 2.5

def read_points():
	file_path = '/home/anshu/f1tenth_gym_ws/src/f1tenth_gym_ros-multi_node/scripts/new/stanley_waypoints.csv'
	with open(file_path) as f:
		path_points = [tuple(line) for line in csv.reader(f)]
	return path_points

def find_distance(x1,y1):
	global xc,yc,yaw,waypoints
	distance = math.sqrt((x1-xc)**2 + (y1-yc)**2)
	return distance

def pose_callback(data):
	global xc,yc,yaw
	# getting x,y
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
	xc = xc + wb*np.cos(yaw)
	yc = yc + wb*np.sin(yaw)

def find_nearest_waypoint():
	global xc,yc,yaw,waypoints,cx,cy

	dx = [xc - icx for icx in cx]
	dy = [yc - icy for icy in cy]
	d = np.hypot(dx, dy)
	nearest_idx = np.argmin(d)

	# Project RMS error onto front axle vector
	front_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]
	error_front_axle = np.dot([dx[nearest_idx], dy[nearest_idx]], front_axle_vec)


	return nearest_idx, error_front_axle


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
	"""
	Plot arrow
	"""
	if not isinstance(x, float):
		for ix, iy, iyaw in zip(x, y, yaw):
			plot_arrow(ix, iy, iyaw)
	else:
		plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
				  fc=fc, ec=ec, head_width=width, head_length=width)
		plt.plot(x, y)
		patches.Rectangle((xc,yc), 0.35,0.2)

def findCircleR(x1, y1, x2, y2, x3, y3) :

	x12 = x1 - x2;	x13 = x1 - x3;  y12 = y1 - y2;  y13 = y1 - y3;  y31 = y3 - y1;  y21 = y2 - y1;  x31 = x3 - x1;  x21 = x2 - x1

	# x1^2 - x3^2
	sx13 = math.pow(x1, 2) -  math.pow(x3, 2)

	# y1^2 - y3^2
	sy13 =  math.pow(y1, 2) -  math.pow(y3, 2)

	sx21 =  math.pow(x2, 2) -  math.pow(x1, 2)
	sy21 =  math.pow(y2, 2) -  math.pow(y1, 2)
	try:
		f = (((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) +  (sy21) * (x13)) // (2 * ((y31) * (x12) - (y21) * (x13))))
		g = (((sx13) * (y12) + (sy13) * (y12) +  (sx21) * (y13) + (sy21) * (y13)) //  (2 * ((x31) * (y12) - (x21) * (y13))))
	except ZeroDivisionError:
		f = 0.1  # Arbitrary high constant
		g = 0.1  # Arbitrary high constant
	c = (- math.pow(x1, 2) -  math.pow(y1, 2) -  2 * g * x1 - 2 * f * y1)

	# eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
	# where centre is (h = -g, k = -f) and
	# radius r as r^2 = h^2 + k^2 - c
	h = -g;	k = -f
	sqr_of_r = h * h + k * k - c

	# r is the radius
	r = round(math.sqrt(sqr_of_r), 5)

	#print("Centre = (", h, ", ", k, ")");
	return r


def normalize_angle(angle):
	"""
	Normalize an angle to [-pi, pi].
	:param angle: (float)
	:return: (float) Angle in radian in [-pi, pi]
	"""
	while angle > np.pi:
		angle -= 2.0 * np.pi

	while angle < -np.pi:
		angle += 2.0 * np.pi

	return angle


def stanley_control(last_idx, linear_vel):
	global xc,yc,yaw,waypoints
	k = .5
	nearest_idx, error_front_axle = find_nearest_waypoint()
	print(last_idx)
	print(nearest_idx)

	if last_idx >= nearest_idx:
		nearest_idx = last_idx


	x1=float(waypoints[nearest_idx][0]);y1=float(waypoints[nearest_idx][1])
	x2=float(waypoints[nearest_idx+1][0]);y2=float(waypoints[nearest_idx+1][1])
	idx_itr = 0
	while (x1 == x2 and y1 == y2):
		x2=float(waypoints[nearest_idx+idx_itr][0]);y2=float(waypoints[nearest_idx+idx_itr][1])
		idx_itr += 1

	psi_t = np.arctan2(y2 - y1, x2 - x1)

	# theta_e corrects the heading error
	theta_e = normalize_angle(psi_t - yaw)
	# theta_d corrects the cross track error
	theta_d = np.arctan2(k * error_front_axle, linear_vel)
	# Steering control
	delta = theta_e + theta_d
	return delta, nearest_idx


if __name__=='__main__':
	rospy.init_node('stanley')
	r = rospy.Rate(10)
	print "RUNNING STALEY.. \n\n"
	time.sleep(2)
	waypoints = read_points()

	global xc,yc,yaw,waypoints,show_animation,VEL_LOOKAHEAD_IDX,cx,cy
	cx = []; cy = []
	for point in waypoints:
		cx.append(float(point[0]))
		cy.append(float(point[1]))


	while not rospy.is_shutdown():
		rospy.Subscriber("/ego_id/odom", Odometry, pose_callback) # Change this topic and it's callback for real world implementation
	#rospy.Subscriber("/racecar_position_gazebo", PoseStamped, pose_callback, queue_size=1) # Change this topic and it's callback for real world implementation
		pub = rospy.Publisher("/ego_id/drive", AckermannDriveStamped, queue_size=10) # velocity and angle

		nearest_idx, _ = find_nearest_waypoint()

		# velocity controller (Longitudinal)
		# Gathering Data
		len_waypoints = len(waypoints)
		driven_distance = 320 #approx calculation - you can calcualte the exact distance travelled by car using odometry data)
		distance_per_waypoint = driven_distance/len_waypoints #comes out to be 320/156 = 2.05 m per waypoint
		# using this data,we can say that to optimize the curvature estimate, we can rely on 'seeing points' 2 waypoints away
		# (At a total distance of ~4 meters, which seems reasonable for the car)
		# find radius of circle passing through these points

		if nearest_idx < (len_waypoints - VEL_LOOKAHEAD_IDX):
			vel_look_idx = nearest_idx + VEL_LOOKAHEAD_IDX
			xcoord1 = float(waypoints[vel_look_idx][0]); ycoord1 = float(waypoints[vel_look_idx][1])
			xcoord2 = float(waypoints[vel_look_idx+1][0]); ycoord2 = float(waypoints[vel_look_idx+1][1])
			xcoord3 = float(waypoints[vel_look_idx+2][0]); ycoord3 = float(waypoints[vel_look_idx+2][1])
			path_radius = findCircleR(xcoord1,ycoord1,xcoord2,ycoord2,xcoord3,ycoord3)
			# if radius is less, speed should be less (as it is a steep turn)
			linear_vel = np.clip(0.6*path_radius,0,MAX_VEL)
		else:
			linear_vel = MAX_VEL

		delta, nearest_idx = stanley_control(nearest_idx,linear_vel)

		# Publishing data
		drive_msg = AckermannDriveStamped()
		drive_msg.drive.speed = linear_vel
		drive_msg.drive.steering_angle = delta
		pub.publish(drive_msg)
		time.sleep(0.1)
		print(drive_msg.drive.steering_angle, 'steer')
		if show_animation:
			plt.cla()
			# for stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
			plot_arrow(xc, yc, yaw)
			plt.plot(cx, cy, "-r", label="course")
			plt.plot(xc, yc, "-b", label="trajectory")
			plt.axis("equal")
			plt.grid(True)
			plt.pause(0.00001)

		# r.sleep()
