from turtle import pos, position
import numpy as np
import rospy  
import math   
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from scipy.spatial import KDTree
import copy

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d

positions = np.array([0.0,0.0,0.0])
pos_drones = np.array([0.0,0.0,0.0])
prev_positions = np.array([0.0,0.0,0.0])

metres_ahead = 4.0

def equidist_path(path,total_path_points):
	resolution = 0.1
	t = np.linspace(0, total_path_points, total_path_points)/ 50

	smallS = 10000
	
	factor = 10
	t_new = np.linspace(0, total_path_points*factor, total_path_points*factor)/ (50*factor)

	x,y = path[:,0],path[:,1]


	# print (t.shape, x.shape)
	tck = interpolate.splrep(t, x, s = smallS, k = 5)
	x_new = interpolate.splev(t_new, tck, der=0)


	tck = interpolate.splrep(t, y, s = smallS, k = 5)
	y_new = interpolate.splev(t_new, tck, der=0)

	distance = np.cumsum(np.sqrt( np.ediff1d(x_new, to_begin=0)**2 + np.ediff1d(y_new, to_begin=0)**2 ))

	n_points = int( distance[-1]/resolution )

	distance = distance/distance[-1]


	fx, fy = interp1d( distance, x_new ), interp1d( distance, y_new )

	alpha = np.linspace(0, 1, n_points)
	x_regular, y_regular = fx(alpha), fy(alpha)

	x_regular = np.expand_dims(x_regular, axis=-1)
	y_regular = np.expand_dims(y_regular, axis=-1)


	path_final = np.concatenate([x_regular, y_regular], axis = 1)

	path = path_final

	# plt.show()
	return path
	

path = np.load("ugv_waypoints.npy")
total_path_points = (path[:,0]).size
path = equidist_path(path,total_path_points)

def odomfunc(odom):
	
	global positions,prev_positions
	# print("ENTERED")
	positions[0] = odom.car_state.pose.pose.position.x
	positions[1] = odom.car_state.pose.pose.position.y
	positions[2] = odom.car_state.pose.pose.position.z
	if abs(positions[0]) < 0.001:
		# print("PARAKH OP")
		# print("BEFORE _ Positions",positions, "Prev_positions" ,prev_positions)
		positions = copy.deepcopy(prev_positions)
		# print("AFTER _ Positions :", positions, "Prev_positions", prev_positions)
	else:
		# print("ELSE")
		prev_positions = copy.deepcopy(positions)

def vfunc(odom_d):
	# print("Entered")
	
	global pos_drones
	
	pos_drones[0] = odom_d.pose.pose.position.x
	pos_drones[1] = odom_d.pose.pose.position.y
	pos_drones[2] = odom_d.pose.pose.position.z 

flag_move_drone = 0
def main():
	global positions,pos_drones,flag_move_drone,prev_positions
	rospy.init_node('high_level_velocity_control',anonymous=True)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)  
	rospy.Subscriber('/mavros/local_position/odom' , Odometry, vfunc)   
	
	
	instance = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) 
	pub1 = rospy.Publisher("uav_wp", PoseStamped, queue_size=1000000) 

	rate = rospy.Rate(5)
	rate.sleep()
	msg = PoseStamped()
																																														 
	while not rospy.is_shutdown():
		# if abs(positions[0]) < 0.001:
		# 	# print("PARAKH OP")
		# 	# print("BEFORE _ Positions",positions, "Prev_positions" ,prev_positions)
		# 	positions = copy.deepcopy(prev_positions)
		# 	# print("AFTER _ Positions :", positions, "Prev_positions", prev_positions)
		# else:
		# 	# print("ELSE")
		# 	prev_positions = copy.deepcopy(positions)
		#if drone behind car and distance>7m, set new wp = car_closest point on path+70 indexes

		# print("CAR_pos) : ",positions[0],positions[1])
		positions_copy_0 = copy.deepcopy(positions[0])
		positions_copy_1 = copy.deepcopy(positions[1])
		pos_drones_copy_0 = copy.deepcopy(pos_drones[0])
		pos_drones_copy_1 = copy.deepcopy(pos_drones[1])
		index_drone = KDTree(path).query(np.array([pos_drones_copy_0,pos_drones_copy_1]))[1]
		index_car =  KDTree(path).query(np.array([positions_copy_0,positions_copy_1]))[1]
		# if abs(positions[0]) < 0.001:

		# 	positions = prev_positions.copy()
		# else:
		# 	# print("ELSE")
		# 	prev_positions = positions.copy()
		# if abs(positions[0]) < 0.001:

		# 	positions = prev_positions.copy()
		# 	# print("AFTER _ Positions :", positions, "Prev_positions", prev_positions)
		# else:
		# 	# print("ELSE")
		# 	prev_positions = positions.copy()

		# print("CAR_pos_1",positions[0],positions[1])
		
		# print("index_drone", index_drone)
		# print("index_car", index_car)
		# print(positions[0])
		
		distance  = np.sqrt((positions[0]-pos_drones[0])**2 + (positions[1]-pos_drones[1])**2)

		# print("CAR_pos_2: ",positions[0],positions[1])

		if (index_drone <index_car) and (distance>(metres_ahead-0.2)):
			flag_move_drone = 1
			msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = (path[int(index_car+metres_ahead*11)][0]), (path[int(index_car+metres_ahead*11)][1]), (positions[2]+18) 


		if flag_move_drone == 1:
			pub1.publish(msg)
			print("Moving to new waypoint",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
			# print("moving to",msg.pose.position.x,msg.pose.position.y)
			# print("cur_pos",pos_drones[0],pos_drones[1])
			# print("CAR_pos_4 : ",positions[0],positions[1])
			# print("Live dist : ",np.sqrt((positions[0]-pos_drones[0])**2+(positions[1]-pos_drones[1])**2))
			if (index_drone>index_car) and (np.abs((np.sqrt((positions[0]-pos_drones[0])**2+(positions[1]-pos_drones[1])**2))-metres_ahead)<0.2):
				print("WP follower : dist : ",(np.sqrt((positions[0]-pos_drones[0])**2+(positions[1]-pos_drones[1])**2)))
				flag_move_drone = 0

		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass