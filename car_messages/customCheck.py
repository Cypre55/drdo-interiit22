import rospy
from car_messages import customMessage

def talker():
    pub = rospy.Publisher('custom_chatter', customMessage)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = customMessage()
    msg.car_centre.x = 0.0
    msg.car_centre.y = 0.0

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass