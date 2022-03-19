from turtle import pos
import numpy as np
import rospy  
import math   
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

positions = np.array([0.0,0.0,0.0])
def odomfunc(odom):
	
	global positions
	# print("ENTERED")
	positions[0] = odom.car_state.pose.pose.position.x
	positions[1] = odom.car_state.pose.pose.position.y
	positions[2] = odom.car_state.pose.pose.position.z

def main():
	global positions
	rospy.init_node('high_level_velocity_control',anonymous=True)
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)     
	
	instance = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10) 

	rate = rospy.Rate(20)
	rate.sleep()
	msg = PoseStamped()
																																														 
	while not rospy.is_shutdown():
		msg.pose.position.x,msg.pose.position.y,msg.pose.position.z = (positions[0]+100), (positions[1]+100), (positions[2]+18) 
		print(positions)
		instance.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass