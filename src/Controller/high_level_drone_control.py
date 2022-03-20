from operator import pos
import numpy as np
from numpy import diff
import rospy  
import math   
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d

from scipy.spatial import KDTree

indexes_ahead = 2
positions = np.array([0.0,0.0,0.0])
prev_positions = np.array([0.0,0.0,0.0])
velocities = np.array([0.0,0.0,0.0])
vel_drones =  np.array([0.0,0.0,0.0])
pos_drones =  np.array([0.0,0.0,0.0])
prev_error = 0.0
sum_error = 0.0
diff_error =  0.0

Kp_pose = 20#0.4
Kd_pose = 100000#0.1

Kpx = Kp_pose
Kpy = Kp_pose
Kpz = 0.0

Kdx = Kd_pose
Kdy = Kd_pose
Kdz = 0.0

Kix = 0.000
Kiy = 0.000
Kiz = 0.0


Kvx = 0.0
Kvy = 0.0
Kvz = 0.0

KP = np.array([[Kpx,0,0],
			[0,Kpy,0],
			[0,0,Kpz]])

KD = np.array([[Kdx,0,0],
			[0,Kdy,0],
			[0,0,Kdz]])
			
KI = np.array([[Kix,0,0],
			[0,Kiy,0],
			[0,0,Kiz]])

KV = np.array([[Kvx,0,0],
			[0,Kvy,0],
			[0,0,Kvz]])

def odomfunc(odom):
	
	global positions,velocities
	# print("ENTERED")
	positions[0] = odom.car_state.pose.pose.position.x
	positions[1] = odom.car_state.pose.pose.position.y
	positions[2] = odom.car_state.pose.pose.position.z + 18.0    
	print("CAR", positions)
	velocities[0] = odom.car_state.twist.twist.linear.x
	velocities[1] = odom.car_state.twist.twist.linear.y
	velocities[2] = 0.0		

def vfunc(odom_d):
	# print("Entered")
	
	global vel_drones,pos_drones
	
	pos_drones[0] = odom_d.pose.pose.position.x
	pos_drones[1] = odom_d.pose.pose.position.y
	pos_drones[2] = odom_d.pose.pose.position.z 
	# print("DRONE", pos_drones)
	vel_drones[0] = odom_d.twist.twist.linear.x
	vel_drones[1] = odom_d.twist.twist.linear.y
	vel_drones[2] = 0.0

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

# path = np.load("path_world1_local_coord.npy")
# path = (path.T)[:,0:2]



total_path_points = (path[:,0]).size
path = equidist_path(path,total_path_points)

def main():
	global positions,pos_drones,vel,vel_drones,prev_error,sum_error,diff_error, prev_positions
	rospy.init_node('high_level_velocity_control',anonymous=True)
	# rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)  
	# rospy.Subscriber('/car_state/kalman_complete' , customMessage, odomfunc)  
	rospy.Subscriber('/car_state/filter_complete' , customMessage, odomfunc)    

	
	
	instance = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10) 
	rospy.Subscriber('/mavros/local_position/odom' , Odometry, vfunc)

	rate = rospy.Rate(10)
	rate.sleep()
	msg = Twist()
	                                                                                                                                                                                     
	while not rospy.is_shutdown():
		# print("EARLIER POS", positions)
		# print("PREVIOUS", prev_positions)
		if np.abs(positions[0]) < 0.001:
			# print("PARAKH OP")
			positions = prev_positions.copy()
		else:
			# print("ELSE")
			prev_positions = positions.copy()
		close_index = KDTree(path).query(np.array([positions[0],positions[1]]))[1]
		positions[0],positions[1] = path[close_index+indexes_ahead][0],path[close_index+indexes_ahead][1]
		error = positions-pos_drones
		diff_error = error-prev_error
		error[error>15] = 15
		error[error<-15] = -15

		diff_error[diff_error>2] = 2
		diff_error[diff_error<-2] = -2
		sum_error += error
		vel = (np.matmul(KP,((error).T))) + np.matmul(KD, (diff_error.T))+ np.matmul(KI, (sum_error.T)) + np.matmul(KV,((velocities-vel_drones).T))
		# print("POS_drone", pos_drones)
		# print("----------------")
		# print("pos_car", positions)
		# print("----------------")
		# print("error", error)
		# print("diff_err", diff_error.T)
		# print("vel", vel)
		prev_error = error
		# print(vel)
		check = np.sqrt(vel[0]**2+vel[1]**2)
		if check > 5:
			vel[0] = vel[0]*5/check
			vel[1] = vel[1]*5/check
		msg.linear.x,msg.linear.y,msg.linear.z = vel[0], vel[1], 0.0 #vel[2]
		instance.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass