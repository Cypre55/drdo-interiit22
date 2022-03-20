#!/usr/bin/env python
from turtle import position
import rospy	#to find the nearest point
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion	#for finding yaw from a quaternion
import math
import time
import numpy as np

path_list = []
prius_pos, prius_pos2 = [0,0, 0], [0, 0, 0]
car_length = 3
index = 0
steer_angle = 0
ld=0
yaw = 0
distance = 0
vel = 0


def pos_callback(Odometry):
	global prius_pos,val,yaw,prius_pos_front,vel
	prius_pos[0], prius_pos[1], prius_pos[2] = Odometry.pose.position.x, Odometry.pose.position.y, Odometry.pose.position.x		#Find the position of the car

def pos_callback2(Odometry):
	global prius_pos,val,yaw,prius_pos_front,vel
	index = Odometry.name.index('prius')


	prius_pos2[0], prius_pos2[1], prius_pos2[2] = Odometry.pose[index].position.x, Odometry.pose[index].position.y, Odometry.pose[index].position.z		#Find the position of the car using gazebo



def main():
	# f = open("x_raw_path", "w")
	# a = open("y_raw_path", "w")

	# f2 = open("x_raw_path2", "w")
	# a2 = open("y_raw_path2", "w")

	time_old = time.time()
	rospy.init_node('pure_pursuit',anonymous=True)
	#rospy.Subscriber('/base_pose_ground_truth' , Odometry, pos_callback)	#Car's Odometry data
	rospy.Subscriber('/mavros/local_position/pose' , PoseStamped, pos_callback)	#drone's Odometry data
	rospy.Subscriber('/gazebo/model_states' , ModelStates, pos_callback2)	#drone's Odometry data



	path_taken_x = []
	path_taken_y = []
	path_taken_z = []


	rate=rospy.Rate(10)

	try:
		while not rospy.is_shutdown():
			time_now = time.time()
			if time_now - time_old >0.5:
				print(prius_pos2[0])
				# f.write(str(prius_pos[0]))
				# f.write(",")
				# a.write(str(prius_pos[1]))
				# a.write(",")

				# f2.write(str(prius_pos2[0]))
				# f2.write(",")
				# a2.write(str(prius_pos2[1]))
				# a2.write(",")

				path_taken_x.append(prius_pos2[0])
				path_taken_y.append(prius_pos2[1])
				path_taken_z.append(prius_pos2[2])


				time_old = time_now

		rate.sleep()

	except KeyboardInterrupt:

		print("saving")
		path_taken_X = np.array(path_taken_x)
		path_taken_Y = np.array(path_taken_y)
		path_taken_Z = np.array(path_taken_z)

		path_taken_X = np.expand_dims(path_taken_X, axis=-1)
		path_taken_Y = np.expand_dims(path_taken_Y, axis=-1)
		path_taken_Z = np.expand_dims(path_taken_Z, axis=-1)

		path_taken = np.concatenate([path_taken_X,path_taken_Y, path_taken_Z], axis = 1)

		np.save("path_taken_world3_gazebo_coord", path_taken)

	# path_taken_X = np.array(path_taken_x)
	# path_taken_Y = np.array(path_taken_y)
	# path_taken = np.concatenate([path_taken_X,path_taken_Y], axis = 1)

	# np.save("path_taken_world1_gazebo_coord", path_taken)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
