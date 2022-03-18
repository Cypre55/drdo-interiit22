#!/usr/bin/env python
import rospy
import numpy as np
import math

from drdo_interiit22.msg import customMessage
import matplotlib.pyplot as plt


pose_for_quiver_x = []
pose_for_quiver_y = []
pose_for_quiver_u = []
pose_for_quiver_v = []
kalman_pose_for_quiver_x = []
kalman_pose_for_quiver_y = []
kalman_pose_for_quiver_u = []
kalman_pose_for_quiver_v = []
pubMsg = customMessage()

pub = rospy.Publisher('car_state/kalman_complete', customMessage, queue_size=10)

Xnp = np.zeros((6, 1), dtype = np.float32)

dt = 1 #
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

def odomfunc(odom):

	global x,y,V,theta, pubMsg

	x = odom.car_state.pose.pose.position.x
	y = odom.car_state.pose.pose.position.y

	quaternions =  odom.car_state.pose.pose.orientation

	V = math.sqrt(odom.car_state.twist.twist.linear.x**2 + odom.car_state.twist.twist.linear.y**2)

	curr_pose = [x, y, odom.car_state.twist.twist.linear.x, odom.car_state.twist.twist.linear.y]
	kalman(curr_pose)

	pubMsg = odom
	pubMsg.car_state.header = odom.header
	pubMsg.car_state.pose.pose.position.x = Xnp[0]
	pubMsg.car_state.pose.pose.position.y = Xnp[1]
	pubMsg.car_state.twist.twist.linear.x = Xnp[2]
	pubMsg.car_state.twist.twist.linear.y = Xnp[3]
	# kalman_pose_for_quiver_x.append(Xnp[0])
	# kalman_pose_for_quiver_y.append(Xnp[1])
	# kalman_pose_for_quiver_u.append(Xnp[2])
	# kalman_pose_for_quiver_v.append(Xnp[3])

	x = np.squeeze(kalman_pose_for_quiver_x[-1])
	y = np.squeeze(kalman_pose_for_quiver_y[-1])
	V = math.sqrt(kalman_pose_for_quiver_u[-1]**2 + kalman_pose_for_quiver_v[-1]**2)


def my_mainfunc():
	rospy.init_node('kalman', anonymous=True)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(pubMsg)
		rate.sleep()                                                                                 #rate.sleep() to run odomfunc once




if __name__ == '__main__':

	try:
		my_mainfunc()
	except rospy.ROSInterruptException:
		pass
