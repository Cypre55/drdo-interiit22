#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from tf.transformations import quaternion_from_euler 

from prius_msgs.msg import Control   

flag = 0
def check_if_controller_up(car_inputs):
    global flag
    if car_inputs.shift_gears != 1:
        print("Controller_is_on")
        flag = 1    



def main():




    rospy.init_node('real_data')

    rospy.Subscriber('prius' , Control, check_if_controller_up) 


    pub1 = rospy.Publisher("uav_wp", PoseStamped, queue_size=1000000) 
    rate = rospy.Rate(20)
    i=0

    global flag
    while flag == 0 and not rospy.is_shutdown():
       
        msg = PoseStamped()
        msg.header.seq = i
        msg.pose.position.x = 0  
        msg.pose.position.y = 0
        msg.pose.position.z = 18 
        pub1.publish(msg)
        
        rate.sleep()

    print("exiting")
    
        


 


if __name__ == "__main__":
    main()
