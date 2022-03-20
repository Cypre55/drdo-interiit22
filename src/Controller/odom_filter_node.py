#!/usr/bin/env python
import rospy
import numpy as np
import math
import time

from drdo_interiit22.msg import customMessage
import matplotlib.pyplot as plt

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d


last_pose=[]


def predict_velocity(last_pose):
	global deque_xposes, deque_yposes,x,y,Vx,Vy

	if(len(last_pose) < 10):
		return x, y,Vx,Vy
	else:
		deque_size = 10
		last_pose = np.array(last_pose)
		x_poses = last_pose[-deque_size:][:,0]
		y_poses = last_pose[-deque_size:][:,1]
		# print(x_poses)
		t = 0.1 * np.linspace(0, deque_size, deque_size)								#0.04 is the timestep

		smallS = 1e+18#100000
		# print(t.shape, len(x_poses))
		# time.sleep(1)
		tck = interpolate.splrep(t, x_poses, s = smallS, k = 5)
		x_new = interpolate.splev(t, tck, der=0)								#der 1 gives derivative				
		x_new_vel = interpolate.splev(t, tck, der=1)								#der 1 gives derivative				

		tck = interpolate.splrep(t, y_poses, s = smallS, k = 5)
		y_new = interpolate.splev(t, tck, der=0)
		y_new_vel = interpolate.splev(t, tck, der=1)


		# plt.clf()
		# plt.plot(x_poses,y_poses)
		# plt.plot(x_new,y_new)
		# plt.pause(0.001)

		# return  x_new[-1], y_new[-1],x_new_vel[-1], y_new_vel[-1]
		# return  x_new[-5], y_new[-5],x_new_vel[-5], y_new_vel[-5]

		return  np.average(x_new[-10:]), np.average(y_new[-10:]), np.average(x_new_vel[-10:]), np.average(x_new_vel[-10:])
		# return  np.average(x_poses[-10:]), np.average(y_poses[-10:]), np.average(x_new_vel[-10:]), np.average(x_new_vel[-10:])
	



vx_predictions = []
vy_predictions = []


def odomfunc(odom):
	global x,y,Vx,Vy,theta,car_near_centre
	# print ("OK")
	
	x = odom.car_state.pose.pose.position.x
	y = odom.car_state.pose.pose.position.y

	quaternions =  odom.car_state.pose.pose.orientation

	Vx = odom.car_state.twist.twist.linear.x
	Vy = odom.car_state.twist.twist.linear.y

	car_near_centre = odom.isCarNinety.data
 



def my_mainfunc():
	rospy.init_node('filter', anonymous=True)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)
	
	# pub = rospy.Publisher('car_state/kalman_complete', customMessage, queue_size=10)
	
	instance = rospy.Publisher('/car_state/filter_complete', customMessage, queue_size=10)
	pubMsg = customMessage()

	rate = rospy.Rate(10)
	rate.sleep()

	global x,y,Vx,Vy,theta,car_near_centre
	x_predictions = []
	y_predictions = []
	x_ar = []
	y_ar = []
	vx_predictions = []
	vy_predictions = []
	Vx_ar = []
	Vy_ar = []
	last_pose = []

	position = np.array([x,y])
	
	try:
		while not rospy.is_shutdown():

			if x != 0 and y != 0:
				position = np.array([x,y])
			
				

			last_pose.append(position)
			x_pred,y_pred,vx_pred, vy_pred = predict_velocity(last_pose)
			# x_pred,y_pred,vx_pred, vy_pred = 0.0, 0.0, 0.0, 0.0
			
			
			x_predictions.append(x_pred)
			y_predictions.append(y_pred)
			x_ar.append(x)
			y_ar.append(y)
			vx_predictions.append(vx_pred)
			vy_predictions.append(vy_pred)
			Vx_ar.append(Vx)
			Vy_ar.append(Vy)

			# if vx_pred is not None:
			# 	V = math.sqrt(vx_pred**2 + vy_pred**2)

			
			pubMsg.isCarNinety.data = car_near_centre

			pubMsg.car_state.pose.pose.position.x = x_pred
			pubMsg.car_state.pose.pose.position.y = y_pred
			pubMsg.car_state.twist.twist.linear.x = vx_pred
			pubMsg.car_state.twist.twist.linear.y = vy_pred
			# times = np.arange(0,len(vx_predictions),0.04)
			
			# times = np.linspace(0, len(vx_predictions)**0.1, len(vx_predictions))	
			
			# plt.clf()
			# plt.plot(times, x_ar,color = 'r', alpha = 0.5)
			# plt.plot(times, y_ar,color = 'g',alpha = 0.5)
			# plt.plot(times, x_predictions,color = 'r',alpha = 1, linewidth=2)
			# plt.plot(times, y_predictions,color = 'g',alpha = 1, linewidth=2)
			# plt.plot(x_ar,y_ar, alpha = 0.5)
			# plt.plot(x_predictions,y_predictions, alpha = 1,linewidth=2)

			# plt.grid()
			

			instance.publish(pubMsg)
			rate.sleep()
		# plt.show()                                                                                 #rate.sleep() to run odomfunc once

	except KeyboardInterrupt:
		print('closing')
		# plt.pause(1000)

if __name__ == '__main__':

	try:
		my_mainfunc()
	except rospy.ROSInterruptException:
		pass
