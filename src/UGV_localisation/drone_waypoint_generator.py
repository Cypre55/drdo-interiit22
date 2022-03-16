#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from drdo_interiit22.msg import customMessage
# from geometry_msgs.msg import TwistStamped
from tf.transformations import euler_from_quaternion

# def twistfunc(twist):
#     global vx,vy,vz
#
#     vx = twist.twist.linear.x
#     vy = twist.twist.linear.y
#     vz = twist.twist.linear.z
#     V = math.sqrt(vx**2 + vy**2)
#


def odomfunc(odom):

    global x, y,z, V, theta

    x = odom.car_state.pose.pose.position.x
    print(x)
    y = odom.car_state.pose.pose.position.y
    z = odom.car_state.pose.pose.position.z
    # V = math.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)

    # quaternions =  odom.pose.pose.orientation

    #index = odom.name.index('prius')

    #x = odom.pose[index].position.x
    #y = odom.pose[index].position.y
    #V = math.sqrt(odom.twist[index].linear.x**2 + odom.twist[index].linear.y**2)

    # quaternions =  odom.pose[index].orientation

    # quaternions_list = [quaternions.x,quaternions.y,quaternions.z,quaternions.w]
    # roll,pitch,yaw = euler_from_quaternion(quaternions_list)
    # theta = yaw
    theta = odom.car_state.pose.pose.orientation.x


def real_data():
    global x,y,V,theta
    rospy.init_node('real_data')
    x=None
    y=None
    V=None
    theta=None

    pub1 = rospy.Publisher("uav_wp", PoseStamped, queue_size=1000000)

    rospy.Subscriber('/car_state/complete', customMessage, odomfunc)
    # rospy.Subscriber('/car_state/cat_velocity' , TwistStamped, twistfunc)

    rate = rospy.Rate(10)
    points = []
    i = 0
    while not rospy.is_shutdown():

        msg = PoseStamped()
        msg.header.seq = i
        msg.pose.position.x = x
        print(x)
        msg.pose.position.y = y
        # msg.pose.position.x = car
        # msg.pose.position.y = path_y[i]
        msg.pose.position.z = z # HARD CODED REPLACE
        pub1.publish(msg)
        i+=1


        rate.sleep()


if __name__ == "__main__":
    real_data()
