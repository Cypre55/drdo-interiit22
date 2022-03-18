#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker

input_points = []
output_points = []
mavros_points = []

def input_cb(data):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 4
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    # marker.scale.z = 0.1

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    p = Point()
    p.x = data.point.x
    p.y = data.point.y
    p.z = data.point.z

    input_points.append(p)

    marker.points = input_points

    input_pub.publish(marker)


def output_cb(data):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 4
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    # marker.scale.z = 0.1

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    p = Point()
    p.x = data.pose.position.x
    p.y = data.pose.position.y
    p.z = data.pose.position.z

    output_points.append(p)

    marker.points = output_points

    out_pub.publish(marker)

def mavros_cb(data):
    marker = Marker()

    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 4
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    # marker.scale.z = 0.1

    # Set the color
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    p = Point()
    p.x = data.pose.position.x
    p.y = data.pose.position.y
    p.z = data.pose.position.z

    mavros_points.append(p)

    marker.points = mavros_points

    mavros_pub.publish(marker)


def visual():
    rospy.init_node('Visualizer')

    global input_pub, out_pub, mavros_pub
    input_pub = rospy.Publisher("/input_gps_vis", Marker, queue_size = 2)
    out_pub = rospy.Publisher("/wp_vis", Marker, queue_size = 2)
    mavros_pub = rospy.Publisher("/mavros_vis", Marker, queue_size = 2)

    rospy.Subscriber("/lane_center", PointStamped, input_cb)
    rospy.Subscriber("/uav_wp", PoseStamped, output_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, mavros_cb)

    while not rospy.is_shutdown():
        rospy.spin()





#     marker_pub.publish(marker)

if __name__ == "__main__":
    visual()