#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point, Pose
from std_msgs.msg import Bool
import numpy as np
import tf.transformations
from scipy.interpolate import UnivariateSpline
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import math

# Constants

uav_height = 18.0
PI = 3.1415
reach_threshold = 0.05
new_wp_resolution = 0.5
deque_size = 4

class Node():
    def __init__(self, uav, ugv, index):
        self.UGV = ugv
        self.UAV = uav

        self.parent = -1
        self.index = index

        self.children = []

class Graph():
    def __init__(self):
        self.arr = []
        self.last_nodes = [0]
        self.root = 0
        self.current = 0

def getYawFromSpline():
    t = np.arange(deque_size)

    np_x = np.array([graph.arr[i].UAV.position.x for i in graph.last_nodes])
    np_y = np.array([graph.arr[i].UAV.position.y for i in graph.last_nodes])

    spl_x = UnivariateSpline(t, np_x)
    spl_y = UnivariateSpline(t, np_y)

    point = deque_size-1
    tan = spl_y.derivative()(point)/spl_x.derivative()(point)

    atan = np.arctan(tan)
    other_yaw = 0.0

    if (atan > 0):
        other_yaw = atan - np.pi
    elif (atan < 0):
        other_yaw = atan + np.pi

    quaternion = (current_uav_pose.pose.orientation.x,
    current_uav_pose.pose.orientation.y,
    current_uav_pose.pose.orientation.z,
    current_uav_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    current_yaw = euler[2]


    yaw = 0
    if(abs(current_yaw - atan) < abs(current_yaw - other_yaw)):
        yaw = atan
    else:
        yaw = other_yaw

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    return quaternion


def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def gps_point_forward(data):
    pose_to_wp_angle = math.atan2(data.point.y - current_uav_pose.pose.position.y, data.point.x - current_uav_pose.pose.position.x)
    _, __, y = euler_from_quaternion(current_uav_pose.pose.orientation.x, current_uav_pose.pose.orientation.y, current_uav_pose.pose.orientation.z,current_uav_pose.pose.orientation.w)

    for theta in [pose_to_wp_angle, y]:
        if(theta < 0):
            theta += 2 * np.pi
    if(abs(y - pose_to_wp_angle) < np.pi / 2):
        return True
    elif(2 * np.pi - abs(y - pose_to_wp_angle) < np.pi / 2):
        return True
    else:
        return False


def center_GPS_cb(data):
    global JointTrajectory
    # print("Received GPS point: ({0}, {1}, {2})".format(data.point.x, data.point.y, data.point.z))

    if(gps_point_forward(data)):
        arr_1 = np.array([data.point.x, data.point.y])
        arr_2 = np.array([graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y ])

        dist = np.linalg.norm(arr_1 - arr_2)
        print(dist)
        print(arr_1)
        print(arr_2)
        if (dist > new_wp_resolution):
            uav = Pose()    
            ugv = Pose()

            uav.position.x = data.point.x
            uav.position.y = data.point.y
            uav.position.z = data.point.z + 18.0

            ugv.position.x = data.point.x
            ugv.position.y = data.point.y
            ugv.position.z = data.point.z

            temp = Node(uav, ugv, len(graph.arr))
            graph.arr.append(temp)
            temp.parent = graph.current
            graph.arr[graph.current].children.append(temp.index)
            graph.current = temp.index

            graph.last_nodes.append(graph.current)
            if (len(graph.last_nodes) > deque_size):
                while (len(graph.last_nodes) != deque_size):
                    graph.last_nodes.pop(0)

            if (len(graph.last_nodes) == deque_size):
                quat = getYawFromSpline()
                graph.arr[graph.current].UAV.orientation.x = quat[0]
                graph.arr[graph.current].UAV.orientation.y = quat[1]
                graph.arr[graph.current].UAV.orientation.z = quat[2]
                graph.arr[graph.current].UAV.orientation.w = quat[3]
                graph.arr[graph.current].UGV.orientation.x = quat[0]
                graph.arr[graph.current].UGV.orientation.y = quat[1]
                graph.arr[graph.current].UGV.orientation.z = quat[2]
                graph.arr[graph.current].UGV.orientation.w = quat[3]
            
            print("Added Node point: ({0}, {1}, {2})".format(graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y, graph.arr[graph.current].UAV.position.z))
            uav_wp.pose = graph.arr[graph.current].UAV
            uav_wp_pub.publish(uav_wp)
            get_GPS_flag.data = False
            get_GPS_pub.publish(get_GPS_flag)

def current_uav_pose_cb(data):
    global current_uav_pose
    current_uav_pose = data

    # Reached Waypoint
    arr_1 = np.array([uav_wp.pose.position.x, uav_wp.pose.position.y])
    arr_2 = np.array([current_uav_pose.pose.position.x, current_uav_pose.pose.position.y])

    dist = np.linalg.norm(arr_1 - arr_2)
    if (dist < reach_threshold):
        get_GPS_flag.data = True
        get_GPS_pub.publish(get_GPS_flag)

    arr_1 = np.array([end_ugv.position.x, end_ugv.position.y])
    dist = np.linalg.norm(arr_1 - arr_2)
    if (dist < reach_threshold):
        end_reached_flag.data = True
        end_reached_mapping_pub.publish(end_reached_flag)




def graph_planner():
    rospy.init_node('graph_planner')

    global uav_wp_pub, get_GPS_pub, end_reached_mapping_pub
    uav_wp_pub = rospy.Publisher("/uav_wp", PoseStamped, queue_size = 100)
    get_GPS_pub = rospy.Publisher("/get_GPS", Bool, queue_size = 100)
    end_reached_mapping_pub = rospy.Publisher("/end_reached_mapping", Bool, queue_size = 100)


    global get_GPS_flag, end_reached_flag
    get_GPS_flag = Bool()
    get_GPS_flag.data = True
    end_reached_flag = Bool()
    end_reached_flag.data = False

    center_GPS_sub = rospy.Subscriber("/lane/mid", JointTrajectory, center_GPS_cb)
    current_uav_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, current_uav_pose_cb)

    global graph
    graph = Graph()

    global uav_wp
    uav_wp = PoseStamped()

    global start_uav, start_ugv
    start_uav = Pose()
    start_ugv = Pose()
    node = Node(start_uav, start_ugv, len(graph.arr))
    graph.arr.append(node)

    global end_ugv
    end_ugv = Pose()

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    graph_planner()