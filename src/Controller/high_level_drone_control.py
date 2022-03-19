from operator import pos
import numpy as np
from numpy import diff
import rospy  
import math   
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

positions = np.array([0.0,0.0,0.0])
prev_positions = np.array([0.0,0.0,0.0])
velocities = np.array([0.0,0.0,0.0])
vel_drones =  np.array([0.0,0.0,0.0])
pos_drones =  np.array([0.0,0.0,0.0])
prev_error = 0.0
sum_error = 0.0
diff_error =  0.0

Kp_pose = 1#0.55
Ki_pose = 0
Kd_pose = 0.2#0.1

Kpx = Kp_pose
Kpy = Kp_pose
Kpz = 0.0

Kdx = Kd_pose
Kdy = Kd_pose
Kdz = 0.0

Kix = Ki_pose
Kiy = Ki_pose
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
	# print("CAR", positions)
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
		error = positions-pos_drones
		diff_error = error-prev_error
		error[error>15] = 15
		error[error<-15] = -15

		diff_error[diff_error>2] = 2
		diff_error[diff_error<-2] = -2
		sum_error += error
		vel = (np.matmul(KP,((error).T))) + np.matmul(KD, (diff_error.T))+ np.matmul(KI, (sum_error.T)) + np.matmul(KV,((velocities-vel_drones).T))
		print("POS_drone", pos_drones)
		print("----------------")
		print("pos_car", positions)
		print("----------------")
		print("error", error)
		print("diff_err", diff_error.T)
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
