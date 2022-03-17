import numpy as np
import rospy  
import math   
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

positions = np.array([0,0,0])
velocities = np.array([0,0,0])
vel_drones =  np.array([0,0,0])
pos_drones =  np.array([0,0,0])
Kpx = 1
Kpy = 1
Kpz = 1
Kdx = 0.1
Kdy = 0.1
Kdz = 1

KP = np.array([[Kpx,0,0],
			[0,Kpy,0],
			[0,0,Kpz]])

KD = np.array([[Kdx,0,0],
			[0,Kdy,0],
			[0,0,Kdz]])

def odomfunc(odom):
	
	global positions,velocities
	positions[0] = odom.car_state.pose.pose.position.x
	positions[1] = odom.car_state.pose.pose.position.y
	positions[2] = odom.car_state.pose.pose.position.z + 18    

	velocities[0] = odom.car_state.twist.twist.linear.x
	velocities[1] = odom.car_state.twist.twist.linear.y
	velocities[2] = 0		

def vfunc(odom_d):
	
	global vel_drones,pos_drones
	pos_drones[0] = odom_d.pose.pose.position.x
	pos_drones[1] = odom_d.pose.pose.position.y
	pos_drones[2] = odom_d.pose.pose.position.z 

	vel_drones[0] = odom_d.twist.twist.linear.x
	vel_drones[1] = odom_d.twist.twist.linear.y
	vel_drones[2] = 0


def main():
	global positions,pos_drones,vel,vel_drones
	rospy.init_node('high_level_velocity_control',anonymous=True)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)  
	instance = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10) 
	rospy.Subscriber('/mavros/local_position/odom' , Odometry, vfunc)

	rate = rospy.Rate(10)
	                                                                                                                                                                                     
	while not rospy.is_shutdown():
		vel = (np.matmul(KP,((positions-pos_drones).T))) + np.matmul(KD,((velocities-vel_drones).T))
		print(vel)
		msg = Twist()
		msg.linear.x,msg.linear.y,msg.linear.z = vel[0], vel[1], 0  #vel[2]
		instance.publish(msg)
	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass