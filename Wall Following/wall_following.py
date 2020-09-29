#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
front_dist = 0
left_dist = 0
right_dist = 0

def average(distance_list):
    sum =0
    for i in distance_list:
        sum += i
    return sum/len(distance_list)

def call_back(msg):
    global front_dist
    global left_dist
    global right_dist
    distance_front = []
    distance_left = []
    distance_right = []
    count = 0
    i = 0
    while (i < len(msg.ranges)):
        if (i >= 480 and i <= 600 and msg.ranges[i] != float('inf')):
            distance_front.append(msg.ranges[i])
        elif (i< 420 and i>= 220 and msg.ranges[i] != float('inf')):
            distance_left.append(msg.ranges[i])
        elif (i > 660 and i<= 860 and msg.ranges[i] != float('inf')):
            distance_right.append(msg.ranges[i])
        i += 4
    front_dist = min(distance_front)
    left_dist = average(distance_left)
    right_dist = average(distance_right)

def control():
    # if (front_dist >= 1.5):
	vel_msg.drive.speed = 4
	p_steer = 0.1
    # if (front_dist < 1.5):
    #     vel_msg.drive.speed = 2
    #     p_steer = 0.2
	steer = left_dist-right_dist
	vel_msg.drive.steering_angle = -p_steer*steer

	vel_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('f1_tenth_wall_following')
    vel_pub = rospy.Publisher('ego_id/drive', AckermannDriveStamped, queue_size=10)
    scan_sub = rospy.Subscriber('ego_id/scan', LaserScan, call_back)
    vel_msg = AckermannDriveStamped()

while not rospy.is_shutdown():
    control()
    rate = rospy.Rate(10)
    # rate.sleep()
    rate.sleep()