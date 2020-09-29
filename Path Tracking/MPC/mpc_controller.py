#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
import cvxpy
import math
import matplotlib.pyplot as plt
from matplotlib import patches

#Global Variables Initilization
x_current = 0
y_current = 0
v_current = 0
yaw_current = 0
# yaw_rate_current = 0

TARGET_SPEED = 3 # velocity m/s
MAX_SPEED = 20  # velocity m/s
MIN_SPEED = 0  # velocity m/s
MAX_ACCEL = 5.0  # maximum accel [m/ss]
STOP_SPEED = 0.5 / 3.6  # stop speed

NX = 4 # State: x, y, yaw, v
NU = 2 # Input u = [a, delta]
T = 2 # Time horizon
# T = 5
DT = 0.05 # Time step
dl = 0.005 # Average ditance between two points

# iterative paramter
MAX_ITER = 4  # Max iteration
DU_TH = 0.1  # iteration finish param
N_IND_SEARCH = 10

#Vehicle Parameters
MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(3.0)  # maximum steering speed [rad/s]
WB = 0.3302  # [m]

# MPC parameters
R = np.diag([1.0, 1.50])  # input cost matrix
Rd = np.diag([3.0, 3.0])  # input difference cost matrix
Q = np.diag([2.0, 2.0, 1.50, 0.0])  # state cost matrix
# R = np.diag([0.01, 0.01])  # input cost matrix
# Rd = np.diag([0.01, 1.0])  # input difference cost matrix
# Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
show_animation = True


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def read_points():
    """
    Read Waypoints
    """
    file_path = '/home/anshu/f1tenth_gym_ws/src/f1tenth_gym_ros-multi_node/scripts/new/mpc_waypoints.csv'
    with open(file_path) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
    return path_points

def make_lists(waypoints):
    """
    Make Waypoints List
    """
    cx = []; cy = []; cyaw = []; cspeed = []
    for i, point in enumerate(waypoints):
        if i+1 == len(waypoints):
            break
        if (point[0] == waypoints[i+1][0] and point[1] == waypoints[i+1][1]):
            pass
        else:
            cx.append(float(point[0]))
            cy.append(float(point[1]))
            cyaw.append(float(point[2]))
            cspeed.append(float(point[3]))
    return cx, cy, cyaw, cspeed

def pose_callback(data):
    global x_current, y_current, v_current, yaw_current, yaw_rate_current
    x_current = data.pose.pose.position.x
    y_current = data.pose.pose.position.y
    # getting yaw
    qx=data.pose.pose.orientation.x
    qy=data.pose.pose.orientation.y
    qz=data.pose.pose.orientation.z
    qw=data.pose.pose.orientation.w
    quaternion = (qx,qy,qz,qw)
    euler = euler_from_quaternion(quaternion)
    yaw_current = euler[2]
    v_current = data.twist.twist.linear.x
    yaw_rate_current = data.twist.twist.angular.z

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        
        move_direction = math.atan2(dy, dx)
        
        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0
        
        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed
        # speed_profile[i] = target_speed
    speed_profile[-1] = 0.0

    return speed_profile

def calc_nearest_index(cx, cy, cyaw, pind):

    dx = [x_current - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [y_current - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - x_current
    dyl = cy[ind] - y_current

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def calc_ref_trajectory(cx, cy, cyaw, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)
    ind, _ = calc_nearest_index(cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0
    for i in range(T + 1):
        travel += abs(v_current) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0
        # print(ind + dind, "index+dinedex")
        # print('\n')

    return xref, ind, dref

def update_state(state, a, delta):
    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER
    state[0] = state[0] + state[3] * math.cos(state[2]) * DT
    state[1] = state[1] + state[3] * math.sin(state[2]) * DT
    state[2] = state[2] + state[3] / WB * math.tan(delta) * DT
    state[3] = state[3] + a * DT

    if state[3] > MAX_SPEED:
        state[3] = MAX_SPEED
    elif state[3] < MIN_SPEED:
        state[3] = MIN_SPEED
    return state


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]
    state = [x0[0], x0[1], x0[3], x0[2]]
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state[0]
        xbar[1, i] = state[1]
        xbar[2, i] = state[3]
        xbar[3, i] = state[2]

    return xbar

def get_linear_model_matrix(v, phi, delta):
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control
    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """
    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """
    if oa is None or od is None:
        oa = [0.0] * T # 1 by 5
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)

        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov

def check_goal(goal, tind, nind):

    # check goal
    dx = x_current - goal[0]
    dy = y_current - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(v_current) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False

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
        patches.Rectangle((x_current,y_current), 0.35,0.2)

def mpc_init(x_list, y_list, yaw_list, v_list, sp):
    global yaw_current

    goal = [x_list[-1], y_list[-1]]

    # initial yaw compensation
    if yaw_current- yaw_list[0] >= math.pi:
        yaw_current -= math.pi * 2.0
    elif yaw_current - yaw_list[0] <= -math.pi:
        yaw_current += math.pi * 2.0

    target_ind, _ = calc_nearest_index(x_list, y_list, yaw_list, 0)

    odelta, oa = None, None

    while not rospy.is_shutdown():
        xref, target_ind, dref = calc_ref_trajectory(x_list , y_list, yaw_list, sp, dl, target_ind)
        x0 = [x_current, y_current, v_current, yaw_current]  # current state

        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        # x0 = update_state(x0, ai, di)

        drive_msg.drive.speed = v_current + ai*DT
        drive_msg.drive.steering_angle = di
        pub.publish(drive_msg)
        # time.sleep(0.2)

        if check_goal(goal, target_ind, len(x_list)):
            print("Goal")
            break

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(x_current, y_current, yaw_current)
            plt.plot(x_list, y_list, "-r", label="course")
            plt.plot(x_current, y_current, "-b", label="trajectory")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.00001)

if __name__ == '__main__':

    # Reading waypoints and making lists
    waypoints = read_points()
    x_list, y_list, yaw_list, v_list  = make_lists(waypoints)

    # Initilization of ROS
    rospy.init_node('MPC')
    rospy.Subscriber("/ego_id/odom", Odometry, pose_callback) # Change this topic and it's callback for real world implementation
    pub = rospy.Publisher("/ego_id/drive", AckermannDriveStamped, queue_size=10) # velocity and angle
    drive_msg = AckermannDriveStamped()

    # Calculate speed profile for the path
    sp = calc_speed_profile(x_list, y_list, yaw_list, TARGET_SPEED)

    # Code for MPC
    mpc_init(x_list, y_list, yaw_list, v_list, sp)



    rate = rospy.sleep(10)
