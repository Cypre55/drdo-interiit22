#!/usr/bin/env python

from re import X
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import numpy as np
import math
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion

from drdo_interiit22.msg import customMessage


from gazebo_msgs.msg import ModelStates

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt

pi = math.pi
inf = np.inf

cross_track_error = []


x,y,V,theta,throttle,steer_input = 0, 0, 0, 0, 0, 0                  # (x,y,V,theta) will store the current position,current speed and orientation
																			# throttle and steer_input will store the inputs to the vehicle

total_path_points = 0
path = None

flag = 0

pose_for_quiver_x = []
pose_for_quiver_y = []
pose_for_quiver_u = []
pose_for_quiver_v = []
kalman_pose_for_quiver_x = []
kalman_pose_for_quiver_y = []
kalman_pose_for_quiver_u = []
kalman_pose_for_quiver_v = []

def equidist_path(path,total_path_points):

	# global total_path_points,path

	resolution = 0.1
	t = np.linspace(0, total_path_points, total_path_points)/ 50

	smallS = 10000
	# print(t)
	factor = 10
	t_new = np.linspace(0, total_path_points*factor, total_path_points*factor)/ (50*factor)

	x,y = path[:,0],path[:,1]


	print (t.shape, x.shape)
	tck = interpolate.splrep(t, x, s = smallS, k = 5)
	x_new = interpolate.splev(t_new, tck, der=0)


	tck = interpolate.splrep(t, y, s = smallS, k = 5)
	y_new = interpolate.splev(t_new, tck, der=0)
	# print(len(path_x))
	# print(x_new.shape)


	distance = np.cumsum(np.sqrt( np.ediff1d(x_new, to_begin=0)**2 + np.ediff1d(y_new, to_begin=0)**2 ))

	n_points = int( distance[-1]/resolution )

	distance = distance/distance[-1]


	fx, fy = interp1d( distance, x_new ), interp1d( distance, y_new )

	alpha = np.linspace(0, 1, n_points)
	x_regular, y_regular = fx(alpha), fy(alpha)

	x_regular = np.expand_dims(x_regular, axis=-1)
	y_regular = np.expand_dims(y_regular, axis=-1)


	path_final = np.concatenate([x_regular, y_regular], axis = 1)
	# plt.plot(path[:,0],path[:,1], color = 'r')
	# plt.plot(path_final[:,0],path_final[:,1], color = 'b')
	path = path_final

	# plt.show()
	return path


def pathfunc():

	global flag
	if flag == 0:
		flag = 1
		global total_path_points,path
		if total_path_points == 0:
			path = np.load("ugv_waypoints.npy")
			total_path_points = (path[:,0]).size

			path = equidist_path(path,total_path_points)



Xnp = np.zeros((6, 1), dtype = np.float32)


dt = 0.1 #
F = np.array(  [[1, 0, dt,  0, 0.5*dt*dt,         0],
				[0, 1,  0, dt,         0, 0.5*dt*dt],
				[0, 0,  1,  0,        dt,         0],
				[0, 0,  0,  1,         0,        dt],
				[0, 0,  0,  0,         1,         0],
				[0, 0,  0,  0,         0,         1]], dtype=np.float32) #transition matrix
u = np.zeros((6, 1), dtype=np.float32) #transition offset
B = np.array([[1, 0, 0, 0, 0, 0],
			  [0, 1, 0, 0, 0, 0],
			  [0, 0, 1, 0, 0, 0],
			  [0, 0, 0, 1, 0, 0],
			  [0, 0, 0, 0, 1, 0],
			  [0, 0, 0, 0, 0, 1]], dtype=np.float32)

H = np.array([[1, 0, 0, 0, 0, 0],
				[0, 1, 0, 0, 0, 0],
				[0, 0, 1, 0, 0, 0],
				[0, 0, 0, 1, 0, 0]], dtype=np.float32)
P = np.array(  [[1, 0, 0, 0, 0, 0],
				[0, 1, 0, 0, 0, 0],
				[0, 0, 1, 0, 0, 0],
				[0, 0, 0, 1, 0, 0],
				[0, 0, 0, 0, 1, 0],
				[0, 0, 0, 0, 0, 1]], dtype=np.float32)
R = np.array(  [[100, 0, 0, 0],
				[0, 100, 0, 0],
				[0, 0, 100, 0],
				[0, 0, 0, 100]], dtype=np.float32) # measurement uncertainty

Q = np.array(  [[0.5, 0, 0, 0, 0, 0],
				[0, 0.5, 0, 0, 0, 0],
				[0, 0, 0.1, 0, 0, 0],
				[0, 0, 0, 0.1, 0, 0],
				[0, 0, 0, 0, 1, 0],
				[0, 0, 0, 0, 0, 1]], dtype=np.float32) # estimate uncertainty
I = np.identity(6, dtype=np.float32)

# Mark 1
# Q *= 0.1
# R *= 100

# Mark 2
# R = np.array(  [[100, 0, 0, 0],
# 				[0, 100, 0, 0],
# 				[0, 0, 100, 0],
# 				[0, 0, 0, 100]], dtype=np.float32) # measurement uncertainty

# Q = np.array(  [[0.5, 0, 0, 0, 0, 0],
# 				[0, 0.5, 0, 0, 0, 0],
# 				[0, 0, 0.1, 0, 0, 0],
# 				[0, 0, 0, 0.1, 0, 0],
# 				[0, 0, 0, 0, 1, 0],
# 				[0, 0, 0, 0, 0, 1]], dtype=np.float32) # estimate uncertainty
# Q *= 1
# R *= 10

Q *= 1
R *= 10

def kalman(z):
	global Xnp, F, u, H, P, R, I, B, Q

	z = np.array(z)
	z = np.expand_dims(z, axis=1)
	y = z - np.matmul(H,Xnp)
	S = np.matmul(H,np.matmul(P,H.T)) + R
	Sinv = np.linalg.inv(S)
	k = np.matmul(P,(np.matmul((H.T),Sinv)))
	Xnp = Xnp + np.matmul(k,y)
	P = np.matmul((I - np.matmul(k,H)),P)

	Xnp = np.matmul(F,Xnp) + np.matmul(B, u)
	P = np.matmul(F,np.matmul(P,F.T)) + Q

first = 0

def odomfunc(odom):

	global x,y,V,theta, Xnp
	global pose_for_quiver_x
	global pose_for_quiver_y
	global pose_for_quiver_u
	global pose_for_quiver_v
	global kalman_pose_for_quiver_x
	global kalman_pose_for_quiver_y
	global kalman_pose_for_quiver_u
	global kalman_pose_for_quiver_v

	x = odom.car_state.pose.pose.position.x
	y = odom.car_state.pose.pose.position.y

	global first

	if(first == 0):
		first = 1
		Xnp[0] = x
		Xnp[1] = y


	quaternions =  odom.car_state.pose.pose.orientation

	V = math.sqrt(odom.car_state.twist.twist.linear.x**2 + odom.car_state.twist.twist.linear.y**2)

	curr_pose = [x, y, odom.car_state.twist.twist.linear.x, odom.car_state.twist.twist.linear.y]


	kalman(curr_pose)

	pose_for_quiver_x.append(curr_pose[0])
	pose_for_quiver_y.append(curr_pose[1])
	pose_for_quiver_u.append(curr_pose[2])
	pose_for_quiver_v.append(curr_pose[3])
	kalman_pose_for_quiver_x.append(Xnp[0])
	kalman_pose_for_quiver_y.append(Xnp[1])
	kalman_pose_for_quiver_u.append(Xnp[2])
	kalman_pose_for_quiver_v.append(Xnp[3])

	quaternions_list = [quaternions.x,quaternions.y,quaternions.z,quaternions.w]
	roll,pitch,yaw = euler_from_quaternion(quaternions_list)
	theta = yaw

def my_mainfunc():
	rospy.init_node('kalman', anonymous=True)
	# rospy.Subscriber('/base_pose_ground_truth' , Odometry, odomfunc)
	# rospy.Subscriber('/mavros/local_position/odom' , Odometry, odomfunc)
	# rospy.Subscriber('/gazebo/model_states' , ModelStates, odomfunc)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)

	path = np.load("ugv_waypoints.npy")
	total_path_points = (path[:,0]).size

	path = equidist_path(path,total_path_points)
	global kalman_pose_for_quiver_y, kalman_pose_for_quiver_x, kalman_pose_for_quiver_u, kalman_pose_for_quiver_v
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		plt.clf()
		plt.plot(pose_for_quiver_x, pose_for_quiver_y, color='r')
		print('orig: ', len(pose_for_quiver_x), len(pose_for_quiver_y))
		try:
			plt.plot(kalman_pose_for_quiver_x, kalman_pose_for_quiver_y, color='b')
		except Exception as e:
			print('kalman: ', len(kalman_pose_for_quiver_x), len(kalman_pose_for_quiver_y))
			if(len(kalman_pose_for_quiver_x) > len(kalman_pose_for_quiver_y)):
				kalman_pose_for_quiver_x = kalman_pose_for_quiver_x[:-1]
			else:
				kalman_pose_for_quiver_y = kalman_pose_for_quiver_y[:-1]
		# plt.xlim(0, 50)
		# plt.ylim(-50, 0)
		# plt.quiver(kalman_pose_for_quiver_x,kalman_pose_for_quiver_y,kalman_pose_for_quiver_u,kalman_pose_for_quiver_v)
		plt.pause(0.0001)
	plt.show()







if __name__ == '__main__':

	try:
		my_mainfunc()
	except rospy.ROSInterruptException:
		pass
