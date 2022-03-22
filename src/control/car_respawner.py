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
    
    rospy.init_node('set_pose')

    rospy.Subscriber('prius' , Control, check_if_controller_up) 


    instance = rospy.Publisher('prius', Control, queue_size=10)

    rate = rospy.Rate(10)

    msg = Control()


  
    state_msg = ModelState()

    state_msg = ModelState()
    state_msg.model_name = 'prius'
    state_msg.pose.position.x = 104.742386
    state_msg.pose.position.y = -101.9010777
    state_msg.pose.position.z = 15.730011

    roll,pitch,yaw = -0.054656, 0.032451, 2.460081

    quarternions = quaternion_from_euler(roll,pitch,yaw)

    state_msg.pose.orientation.x = quarternions[0]
    state_msg.pose.orientation.y = quarternions[1]
    state_msg.pose.orientation.z = quarternions[2]
    state_msg.pose.orientation.w = quarternions[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    


    print("spawned")
    global flag
    while flag == 0 and not rospy.is_shutdown():

     
        msg.throttle = 0                             
        msg.brake = 1 
        msg.steer = 0
        msg.shift_gears =1

        instance.publish(msg)

        

        rate.sleep()

    
    print("exiting")






        
    

if __name__ == "__main__":
    main()
