#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point, Pose
from std_msgs.msg import Bool
import numpy as np
import tf.transformations
from scipy.interpolate import UnivariateSpline
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import matplotlib.pyplot as plt
from drdo_interiit22.msg import customMessage
import math
import time

# Constants
# TODO:
# Starts when took_off
# Start, End Condition

UAV_HEIGHT = 18.0
PI = 3.1415
REACH_THRESHOLD = 0.5
VISITED_RADIUS = 1.8
NEW_WP_RESOLUTION = 5
DEQUE_SIZE = 4


class Node():
    def __init__(self, uav, ugv, index):
        self.UGV = ugv # Pose()
        self.UAV = uav # Pose()

        self.parent = -1
        self.index = index

        self.children = []

class Graph():
    def __init__(self):
        self.arr = []
        self.last_nodes = []
        self.root = 0
        self.current = -1

def getYawFromSpline():
    t = np.arange(DEQUE_SIZE)

    np_x = np.array([graph.arr[i].UAV.position.x for i in graph.last_nodes])
    np_y = np.array([graph.arr[i].UAV.position.y for i in graph.last_nodes])

    spl_x = UnivariateSpline(t, np_x)
    spl_y = UnivariateSpline(t, np_y)

    point = DEQUE_SIZE-1
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

    # if (abs(current_yaw - yaw) < np.pi/4):
    #     yaw = current_yaw

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

    return quaternion


def gps_point_forward(data):
    pose_to_wp_angle = math.atan2(data[1] - current_uav_pose.pose.position.y, data[0] - current_uav_pose.pose.position.x)
    # _, __, y = euler_from_quaternion(current_uav_pose.pose.orientation.x, current_uav_pose.pose.orientation.y, current_uav_pose.pose.orientation.z,current_uav_pose.pose.orientation.w)

    quaternion = (current_uav_pose.pose.orientation.x,
    current_uav_pose.pose.orientation.y,
    current_uav_pose.pose.orientation.z,
    current_uav_pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    y = euler[2]

    for theta in [pose_to_wp_angle, y]:
        if(theta < 0):
            theta += 2 * np.pi
    if(abs(y - pose_to_wp_angle) < np.pi / 2):
        return True
    elif(2 * np.pi - abs(y - pose_to_wp_angle) < np.pi / 2):
        return True
    else:
        return False

def print_graph():
    for node in graph.arr:
        print()
        print("X: ", node.UAV.position.x)
        print("Y: ", node.UAV.position.y)

def in_prev_buffer(point):
    point_xy = point[:2]
    for node in graph.arr:
        node_xy = np.array([node.UAV.position.x, node.UGV.position.y])
        dist = np.linalg.norm(node_xy - point_xy)
        if (dist < VISITED_RADIUS):
            print("In Prev Buff :(:):(")
            return True
    return False

def add_node_to_graph(point):
    global reaching_flag

    if in_prev_buffer(point):
        # point[0] = 2*graph.arr[graph.current].UAV.position.x - graph.arr[graph.last_nodes[-2]].UAV.position.x
        # point[1] = 2*graph.arr[graph.current].UAV.position.y - graph.arr[graph.last_nodes[-2]].UAV.position.y 
        # print("Forcfully moving forward")
        return

    uav = Pose()
    ugv = Pose()

    uav.position.x = point[0]
    uav.position.y = point[1]
    uav.position.z = point[2] + UAV_HEIGHT

    ugv.position.x = point[0]
    ugv.position.y = point[1]
    ugv.position.z = point[2]

    temp = Node(uav, ugv, len(graph.arr))
    graph.arr.append(temp)
    temp.parent = graph.current
    if graph.current != -1:
        graph.arr[graph.current].children.append(temp.index)
    graph.current = temp.index

    graph.last_nodes.append(graph.current)
    if (len(graph.last_nodes) > DEQUE_SIZE):
        while (len(graph.last_nodes) != DEQUE_SIZE):
            graph.last_nodes.pop(0)

    # if (len(graph.last_nodes) == DEQUE_SIZE):
    # 	quat = getYawFromSpline()
    # 	graph.arr[graph.current].UAV.orientation.x = quat[0]
    # 	graph.arr[graph.current].UAV.orientation.y = quat[1]
    # 	graph.arr[graph.current].UAV.orientation.z = quat[2]
    # 	graph.arr[graph.current].UAV.orientation.w = quat[3]
    # 	graph.arr[graph.current].UGV.orientation.x = quat[0]
    # 	graph.arr[graph.current].UGV.orientation.y = quat[1]
    # 	graph.arr[graph.current].UGV.orientation.z = quat[2]
    # 	graph.arr[graph.current].UGV.orientation.w = quat[3]

    
    with open('ugv_waypoints.npy', 'wb') as f:
        UGV_waypoints = [np.array([i.UGV.position.x, i.UGV.position.y, i.UGV.position.z]) for i in graph.arr]
        np.save(f, np.array(UGV_waypoints))

    if len(graph.last_nodes) >= 3:
        prev_node = np.array([graph.arr[graph.last_nodes[-1]].UAV.position.x, graph.arr[graph.last_nodes[-1]].UAV.position.y])
        prev_2_node = np.array([graph.arr[graph.last_nodes[-2]].UAV.position.x, graph.arr[graph.last_nodes[-2]].UAV.position.y])
        prev_3_node = np.array([graph.arr[graph.last_nodes[-3]].UAV.position.x, graph.arr[graph.last_nodes[-3]].UAV.position.y])
        new_vect = prev_node - prev_2_node
        new_vect /= np.linalg.norm(new_vect)
        prev_vect = prev_2_node - prev_3_node
        prev_vect /= np.linalg.norm(prev_vect)

        dot = np.dot(new_vect, prev_vect)
        # print("INNER PREV+: " + str(prev_3_node))
        # print("INNER PREV: " + str(prev_2_node))
        # print("INNER NEW: " + str(prev_node))
        # print("INNER DOT: " + str(dot))

    uav_wp.pose = graph.arr[graph.current].UAV
    uav_wp_pub.publish(uav_wp)
    reaching_flag = True
    # time.sleep(0.1)
    print("Added Node: ({0}, {1}, {2})".format(graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y, graph.arr[graph.current].UAV.position.z))

def check_spline_pts(car_front_, car_back_, right_bound, left_bound, ref_point):
    car_vect_ = car_front_ - car_back_
    car_vect_ = car_vect_/np.linalg.norm(car_vect_)
    
    vect_r = joint_traj[right_bound, :2] - ref_point
    vect_r = vect_r/np.linalg.norm(vect_r)

    vect_l = joint_traj[left_bound, :2] - ref_point
    vect_l = vect_l/np.linalg.norm(vect_l)

    dot_r = np.dot(car_vect_, np.expand_dims(vect_r, axis = -1)).item()
    dot_l = np.dot(car_vect_, np.expand_dims(vect_l, axis = -1)).item()

    print("vect_r", vect_r)
    print("vect_l", vect_l)
    # print()
    print("car_vect_", car_vect_)
    print("Dot product ", dot_r)
    print("Dot product ", dot_l)

    # if dot_r >0 and dot_r > dot_l:
    #     next_graph_pt = joint_traj[right_bound]
    # elif dot_r < 0:
    #     return None
    # elif dot_l >0 and dot_r < dot_l:
    #     next_graph_pt = joint_traj[left_bound]
    # elif dot_l < 0:
    #     return None

    if dot_r > dot_l:
        if dot_r > 0:
            next_graph_pt = joint_traj[right_bound]
        else:
            return None
    else:
        if dot_l > 0:
            next_graph_pt = joint_traj[left_bound]
        else:
            return None
    
    return next_graph_pt

def backtrack_graph():
    global reaching_flag
    # if reaching_flag == True:
    #     return

    if len(graph.last_nodes) >= 3:
        print("Backtracking |!~!|")
        
        uav_wp.pose = graph.arr[graph.last_nodes[-3]].UAV
        uav_wp_pub.publish(uav_wp)
        reaching_flag = True

        
        graph.arr.pop()
        graph.arr.pop()
        # if graph.current != -1:
        #     graph.arr[graph.last_nodes[-2]].children.remove(graph.current)
        graph.last_nodes.pop()
        graph.last_nodes.pop()
        if graph.arr[graph.last_nodes[0]].parent != -1:
            graph.last_nodes.insert(0, graph.arr[graph.last_nodes[0]].parent)
        if graph.arr[graph.last_nodes[0]].parent != -1:
            graph.last_nodes.insert(0, graph.arr[graph.last_nodes[0]].parent)
        graph.current = graph.last_nodes[-1]


def build_graph():
    global direction, car_vect, flag
    try:
        if (joint_traj.shape == (0,)):
            pass
    except Exception as e:
        print("Joint Traj Not Defined")
        return
    try:
        if (current_uav_xyz.shape == (0,)):
            pass
    except Exception as e:
        print("UAVXYZ Not Defined")
        return

    try:
        if (flag == True):
            pass
    except Exception as e:
        flag = False

    if reaching_flag == True:
        return

    try:
        if joint_traj[:, :2].any() == 0:
            pass
    except Exception as e:
        print(e)
        print("Joint Traj: {}".format(joint_traj))
        return

    # plt.clf()
    # plt.scatter(current_uav_xyz[0, 0], current_uav_xyz[0, 1])
    # plt.plot(joint_traj[:, 0], joint_traj[:, 1])

    # print("In Build")
    graph_current_xy = np.array([graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y])
    graph_current_xy.reshape((1,2))

    # current_uav_xyz

    distances = np.linalg.norm(joint_traj[:, :2] - graph_current_xy, axis=1)
    distances_drn = np.linalg.norm(joint_traj[:, :2] - current_uav_xyz[0, :2], axis=1)
    
    index = distances.argmin()
    index_drn = distances_drn.argmin()

    distances_ind = np.linalg.norm(joint_traj[:, :2] - joint_traj[index_drn, :2], axis=1)
    index_ind = distances_ind.argmin()
    # distances = np.linalg.norm(joint_traj[:, :2] - joint_traj[index][:2].reshape((1,2)), axis=1)
    # New Approach

    # index_drn = index

    mask = distances >= NEW_WP_RESOLUTION
    mask_drn = distances_drn >= NEW_WP_RESOLUTION
    mask_ind = distances_ind >= NEW_WP_RESOLUTION

    ## Using Prev Node of Graph as Center
    where_zero = np.where(mask==False)
    ## Using Drone Position as Center
    where_zero_drn = np.where(mask_drn==False)
    ## Using Spline Projection as Center
    where_zero_ind = np.where(mask_ind==False)
    # print(where_zero)

    where_zero_drn = where_zero_ind

    if where_zero_drn[0].shape[0] != 0:
        print("Drone to splin min found")
        left_bound = where_zero_drn[0][0] - 1
        right_bound = where_zero_drn[0][-1] + 1
    else:
        print("Drone to splin min not found !!")
        right_bound = index - 1
        left_bound = index + 1


    # if where_zero[0].shape[0] != 0:
    #     print("Graph to splin min found")

    #     left_bound = where_zero[0][0] - 1
    #     right_bound = where_zero[0][-1] + 1
    # else:
    #     print("Graph to splin min not found !!")

        # right_bound = index - 1
        # left_bound = index + 1
    
    

    if (left_bound < 0):
        left_bound = 0
    if (left_bound >= joint_traj.shape[0]):
        left_bound = joint_traj.shape[0] - 1

    if (right_bound < 0):
        right_bound = 0
    if (right_bound >= joint_traj.shape[0]):
        right_bound = joint_traj.shape[0] - 1

    print(left_bound)
    print(right_bound)

    # print(distances[left_bound])
    # print(np.linalg.norm(joint_traj[left_bound, :2] - graph_current_xy))
    # print(distances[left_bound+1])
    # print(np.linalg.norm(joint_traj[left_bound+1, :2] - graph_current_xy))

    # print(distances[right_bound])
    # print(np.linalg.norm(joint_traj[right_bound, :2] - graph_current_xy))

    # ind_chosen = left_bound

    try:
        if direction == True:
            pass
    except:
        direction = -1
    
    ## To be done only on start
    if (is_car is not None and is_car == True) and car_vect is None:
        next_graph_pt = check_spline_pts(car_front, car_back, right_bound, left_bound, current_uav_xyz[:, :2])
        
        if next_graph_pt is not None:
            car_vect = car_front - car_back
            add_node_to_graph(next_graph_pt)
        else:
            print("Stuck, not added")
        return

    elif len(graph.last_nodes) >=2:
        print("Using dot checking cont.")

        prev_node = np.array([graph.arr[graph.last_nodes[-1]].UAV.position.x, graph.arr[graph.last_nodes[-1]].UAV.position.y])
        prev_2_node = np.array([graph.arr[graph.last_nodes[-2]].UAV.position.x, graph.arr[graph.last_nodes[-2]].UAV.position.y])

        next_graph_pt = check_spline_pts(prev_node, prev_2_node, right_bound, left_bound, current_uav_xyz[:, :2])

        if next_graph_pt is not None:
            add_node_to_graph(next_graph_pt)
        else:
            backtrack_graph()

    else:
        print("Car not visible in frame. !!")

    # plt.scatter(joint_traj[ind_chosen][0], joint_traj[ind_chosen][1], color='b', alpha=0.7, linewidth = 10)
    # plt.scatter(joint_traj[right_bound][0], joint_traj[right_bound][1], color = 'g', alpha=1)
    # plt.scatter(joint_traj[left_bound][0], joint_traj[left_bound][1], color = 'r', alpha = 1)
    
    # plt.scatter(joint_traj[index_drn][0], joint_traj[index_drn][1], color = 'k', alpha = 0.5, linewidth = 5)


    # plt.pause(0.003)

    # if flag == True:
    #     # plt.show()
    #     return
    # print(direction)	
    # add_node_to_graph(joint_traj[ind_chosen])

    ################

    # Other Approach

    # seq = range(index, joint_traj.shape[0])
    # next_seq = reversed(range(index))
    # if direction == 1:
    # 	t = next_seq
    # 	next_seq = seq
    # 	seq = t

    # for i in seq:
        
    # 	dist = np.linalg.norm(joint_traj[i, :2] - graph_current_xy)

    # 	if (dist > NEW_WP_RESOLUTION):

    # 		# Check Direction compared to previous node
    # 		dot = 0
    # 		if len(graph.last_nodes) >= 2:
    # 			prev_node = np.array([graph.arr[graph.last_nodes[0]].UAV.position.x, graph.arr[graph.last_nodes[0]].UAV.position.y])
    # 			prev_2_node = np.array([graph.arr[graph.last_nodes[1]].UAV.position.x, graph.arr[graph.last_nodes[1]].UAV.position.y])
    # 			prev_vect = prev_node - prev_2_node
    # 			new_vect = joint_traj[i, :2].reshape((2,)) - prev_node

    # 			dot = np.dot(new_vect, prev_vect)
    # 			if (dot < 0):
    # 				continue

    # 		if dot != 0:
    # 			print("OUTER: " + str(prev_2_node))
    # 			print("OUTER: " + str(prev_node))
    # 			print("OUTER: " + str(joint_traj[i, :2].reshape((2,))))
    # 			print("OUTER: " + str(dot))
    # 		add_node_to_graph(joint_traj[i])

    # 		return

    # for i in next_seq:
    # 	dist = np.linalg.norm(joint_traj[i, :2] - graph_current_xy)

    # 	if (dist > NEW_WP_RESOLUTION):

    # 		# Check Direction compared to previous node
    # 		dot = 0
    # 		if len(graph.last_nodes) >= 2:
    # 			prev_node = np.array([graph.arr[graph.last_nodes[0]].UAV.position.x, graph.arr[graph.last_nodes[0]].UAV.position.y])
    # 			prev_2_node = np.array([graph.arr[graph.last_nodes[1]].UAV.position.x, graph.arr[graph.last_nodes[1]].UAV.position.y])
    # 			prev_vect = prev_node - prev_2_node
    # 			new_vect = joint_traj[i, :2].reshape((2,)) - prev_node

    # 			dot = np.dot(new_vect, prev_vect)
    # 			if (dot < 0):
    # 				continue

            
    		# add_node_to_graph(joint_traj[i])

    # 		return

    ############

def center_GPS_cb(data):
    global joint_traj
    if data.joint_names[0] == "BACKTRACK":
        print("Recevied Message to BACKTRACK")
        if not reaching_flag:
            backtrack_graph()
            joint_traj = None
        return
    joint_traj = np.array([np.array(i.positions) for i in data.points])

def current_uav_pose_cb(data):
    global current_uav_pose, current_uav_xyz, reaching_flag
    current_uav_pose = data

    current_uav_xyz = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    current_uav_xyz = current_uav_xyz.reshape((3, 1)).T

    if len(graph.arr) == 0:
        # print("Root")
        start_ugv = current_uav_pose.pose
        start_ugv.position.z -= UAV_HEIGHT
        start_ugv = np.array([start_ugv.position.x, start_ugv.position.y, start_ugv.position.z])
        add_node_to_graph(start_ugv)
        reaching_flag = True
        # print("Done Root")
    
    # Reached Waypoint
    arr_1 = np.array([uav_wp.pose.position.x, uav_wp.pose.position.y, uav_wp.pose.position.z])
    arr_2 = np.array([current_uav_pose.pose.position.x, current_uav_pose.pose.position.y, current_uav_pose.pose.position.z])
    dist = np.linalg.norm(arr_1 - arr_2)
    if (dist < REACH_THRESHOLD):
        reaching_flag = False
        print("Reached Waypoint: ({}, {}, {})".format(uav_wp.pose.position.x, uav_wp.pose.position.y, uav_wp.pose.position.z))
        get_GPS_flag.data = True
        get_GPS_pub.publish(get_GPS_flag)

    arr_1 = np.array([end_ugv.position.x, end_ugv.position.y])
    dist = np.linalg.norm(arr_1 - arr_2[:2])
    if (dist < REACH_THRESHOLD):
        end_reached_flag.data = True
        end_reached_mapping_pub.publish(end_reached_flag)	

def car_state_cb(data):
    global is_car, car_back, car_front
    is_car = data.isMaskDetected.data
    car_back = np.array([[data.car_back.x, data.car_back.y]])
    car_front = np.array([[data.car_front.x, data.car_front.y]])


def graph_planner():
    rospy.init_node('graph_planner')

    global uav_wp_pub, get_GPS_pub, end_reached_mapping_pub
    uav_wp_pub = rospy.Publisher("/uav_wp", PoseStamped, queue_size = 100)
    get_GPS_pub = rospy.Publisher("/get_GPS", Bool, queue_size = 100)
    end_reached_mapping_pub = rospy.Publisher("/end_reached_mapping", Bool, queue_size = 100)


    global get_GPS_flag, end_reached_flag, reaching_flag
    reaching_flag = False
    get_GPS_flag = Bool()
    get_GPS_flag.data = True
    end_reached_flag = Bool()
    end_reached_flag.data = False

    center_GPS_sub = rospy.Subscriber("/lane/mid", JointTrajectory, center_GPS_cb)
    current_uav_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, current_uav_pose_cb)
    car_state_sub = rospy.Subscriber("/car_state/complete", customMessage, car_state_cb)

    global graph
    graph = Graph()

    global uav_wp
    uav_wp = PoseStamped()

    global start_uav, start_ugv
    start_uav = Pose()
    start_ugv = Pose()

    global car_back, car_front, is_car, car_vect
    car_vect = None
    car_back = None
    car_front = None
    is_car = None
    # start_uav = current_uav_pose.pose
    # start_ugv = start_uav
    # start_ugv.position.z = start_ugv.position.z - UAV_HEIGHT
    
    global joint_traj
    joint_traj = None

    global end_ugv
    end_ugv = Pose()

    global current_uav_xyz
    current_uav_xyz = None

    # , current_uav_xyz
    # joint_traj_arr = np.array([])
    # current_uav_xyz = np.array([])
    # print(joint_traj_arr.shape)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if joint_traj is not None and current_uav_xyz is not None:


            build_graph()
            is_car = None
            joint_traj = None
            current_uav_xyz = None

        rate.sleep()

        


if __name__ == "__main__":
    graph_planner()