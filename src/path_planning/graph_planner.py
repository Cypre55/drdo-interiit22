#!/usr/bin/python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point, Pose
from std_msgs.msg import Bool
import numpy as np
import tf.transformations
from scipy.interpolate import UnivariateSpline
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from drdo_interiit22.msg import customMessage
import math
import time

# Constants
# TODO:
# Starts when took_off
# Only forward
# Start, End Condition

UAV_HEIGHT = 18.0
PI = 3.1415
REACH_THRESHOLD = 0.05
NEW_WP_RESOLUTION = 3
DEQUE_SIZE = 4


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

def build_graph(joint_traj):
	global direction
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

	graph_current_xy = np.array([graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y])
	graph_current_xy.reshape((1,2))

	distances = np.linalg.norm(joint_traj[:, :2] - graph_current_xy, axis=1)
	
	# New Approach

	mask = distances > NEW_WP_RESOLUTION

	where_zero = np.where(mask==False)
	# print(where_zero)

	left_bound = where_zero[0][0] - 1
	right_bound = where_zero[0][-1] + 1

	

	if (left_bound < 0):
		left_bound = 0
	if (right_bound >= joint_traj.shape[0]):
		right_bound = joint_traj.shape[0] - 1


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
		# 0 left
		# 1 right
		direction = 1
	

	if (is_car != None and is_car == True):
		vect = joint_traj[right_bound, 2] - graph_current_xy
		car_vect = car_front - car_back 
		dot = np.dot(vect, car_vect)
		if dot > 0:
			ind_chosen = right_bound
			direction = 0

	# print(direction)

	# if direction == 1:
	# 	ind_chosen = right_bound
	# else:
	# 	ind_chosen = left_bound

	
	
	# uav = Pose()
	# ugv = Pose()

	# i = ind_chosen
	# # i = 0

	# uav.position.x = joint_traj[i, 0]
	# uav.position.y = joint_traj[i, 1]
	# uav.position.z = joint_traj[i, 2] + UAV_HEIGHT

	# ugv.position.x = joint_traj[i, 0]
	# ugv.position.y = joint_traj[i, 1]
	# ugv.position.z = joint_traj[i, 2]

	# temp = Node(uav, ugv, len(graph.arr))
	# graph.arr.append(temp)
	# temp.parent = graph.current
	# graph.arr[graph.current].children.append(temp.index)
	# graph.current = temp.index

	# graph.last_nodes.append(graph.current)
	# if (len(graph.last_nodes) > DEQUE_SIZE):
	# 	while (len(graph.last_nodes) != DEQUE_SIZE):
	# 		graph.last_nodes.pop(0)

	# # if (len(graph.last_nodes) == DEQUE_SIZE):
	# # 	quat = getYawFromSpline()
	# # 	graph.arr[graph.current].UAV.orientation.x = quat[0]
	# # 	graph.arr[graph.current].UAV.orientation.y = quat[1]
	# # 	graph.arr[graph.current].UAV.orientation.z = quat[2]
	# # 	graph.arr[graph.current].UAV.orientation.w = quat[3]
	# # 	graph.arr[graph.current].UGV.orientation.x = quat[0]
	# # 	graph.arr[graph.current].UGV.orientation.y = quat[1]
	# # 	graph.arr[graph.current].UGV.orientation.z = quat[2]
	# # 	graph.arr[graph.current].UGV.orientation.w = quat[3]

	# print("Added Node point: ({0}, {1}, {2})".format(graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y, graph.arr[graph.current].UAV.position.z))
	# uav_wp.pose = graph.arr[graph.current].UAV
	# uav_wp_pub.publish(uav_wp)
	# time.sleep(1)
	# with open('ugv_waypoints.npy', 'wb') as f:
	# 	UGV_waypoints = [np.array([i.UGV.position.x, i.UGV.position.y, i.UGV.position.z]) for i in graph.arr]
	# 	np.save(f, np.array(UGV_waypoints))

	# return

	################

	# Other Approach
	index = distances.argmin()

	# POSSIBLE POINT OF FAILURE: FORWARD AND BACKWARD
	list  = reversed(range(index))
	if direction == 0:
		list = range(index)
	for i in list:
		
		dist = np.linalg.norm(joint_traj[i, :2] - graph_current_xy)

		if (dist > NEW_WP_RESOLUTION):

			uav = Pose()
			ugv = Pose()

			uav.position.x = joint_traj[i, 0]
			uav.position.y = joint_traj[i, 1]
			uav.position.z = joint_traj[i, 2] + UAV_HEIGHT

			ugv.position.x = joint_traj[i, 0]
			ugv.position.y = joint_traj[i, 1]
			ugv.position.z = joint_traj[i, 2]

			temp = Node(uav, ugv, len(graph.arr))
			graph.arr.append(temp)
			temp.parent = graph.current
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
			# 	graph.arr[graph.currient].UGV.orientation.x = quat[0]
			# 	graph.arr[graph.current].UGV.orientation.y = quat[1]
			# 	graph.arr[graph.current].UGV.orientation.z = quat[2]
			# 	graph.arr[graph.current].UGV.orientation.w = quat[3]

			print("Added Node point: ({0}, {1}, {2})".format(graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y, graph.arr[graph.current].UAV.position.z))
			uav_wp.pose = graph.arr[graph.current].UAV
			uav_wp_pub.publish(uav_wp)
			time.sleep(1)
			with open('ugv_waypoints.npy', 'wb') as f:
				UGV_waypoints = [np.array([i.UGV.position.x, i.UGV.position.y, i.UGV.position.z]) for i in graph.arr]
				np.save(f, np.array(UGV_waypoints))

			return
	############

def center_GPS_cb(data):
	# global joint_traj_arr    
	joint_traj_arr = np.array([np.array(i.positions) for i in data.points])
	build_graph(joint_traj_arr)
	# print(joint_traj_arr.shape)
	# print("Hi")
	# joint_traj_arr = joint_traj_arr[:].positions
	# print(joint_traj_arr[0])

	# print(joint_traj_arr)
	# print("Received GPS point: ({0}, {1}, {2})".format(data.point.x, data.point.y, data.point.z))
	# print(len(joint_traj_arr))
	
	# if(gps_point_forward(data)):
	#     arr_1 = np.array([data.point.x, data.point.y])
	#     arr_2 = np.array([graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y ])

	#     dist = np.linalg.norm(arr_1 - arr_2)
	#     print(dist)
	#     print(arr_1)
	#     print(arr_2)
	#     if (dist > NEW_WP_RESOLUTION):
			# uav = Pose()    
			# ugv = Pose()

			# uav.position.x = data.point.x
			# uav.position.y = data.point.y
			# uav.position.z = data.point.z + 18.0

			# ugv.position.x = data.point.x
			# ugv.position.y = data.point.y
			# ugv.position.z = data.point.z

			# temp = Node(uav, ugv, len(graph.arr))
			# graph.arr.append(temp)
			# temp.parent = graph.current
			# graph.arr[graph.current].children.append(temp.index)
			# graph.current = temp.index

			# graph.last_nodes.append(graph.current)
			# if (len(graph.last_nodes) > DEQUE_SIZE):
			# 	while (len(graph.last_nodes) != DEQUE_SIZE):
			# 		graph.last_nodes.pop(0)

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
			
			# print("Added Node point: ({0}, {1}, {2})".format(graph.arr[graph.current].UAV.position.x, graph.arr[graph.current].UAV.position.y, graph.arr[graph.current].UAV.position.z))
			# uav_wp.pose = graph.arr[graph.current].UAV
			# uav_wp_pub.publish(uav_wp)
			# get_GPS_flag.data = False
			# get_GPS_pub.publish(get_GPS_flag)

def current_uav_pose_cb(data):
	global current_uav_pose, current_uav_xyz
	current_uav_pose = data

	start_uav = current_uav_pose.pose
	start_ugv = start_uav
	start_ugv.position.z -= UAV_HEIGHT

	node = Node(start_uav, start_ugv, len(graph.arr))
	graph.arr.append(node)

	current_uav_xyz = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
	current_uav_xyz = current_uav_xyz.reshape((3, 1)).T
	
	# Reached Waypoint
	arr_1 = np.array([uav_wp.pose.position.x, uav_wp.pose.position.y])
	arr_2 = np.array([current_uav_pose.pose.position.x, current_uav_pose.pose.position.y])
	dist = np.linalg.norm(arr_1 - arr_2)
	if (dist < REACH_THRESHOLD):
		get_GPS_flag.data = True
		get_GPS_pub.publish(get_GPS_flag)

	arr_1 = np.array([end_ugv.position.x, end_ugv.position.y])
	dist = np.linalg.norm(arr_1 - arr_2)
	if (dist < REACH_THRESHOLD):
		end_reached_flag.data = True
		end_reached_mapping_pub.publish(end_reached_flag)	

def car_state_cb(data):
	
	is_car = data.isMaskDetected.data
	car_back = np.array([[data.car_back.x, data.car_back.y]])
	car_front = np.array([[data.car_front.x, data.car_front.y]])


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
	car_state_sub = rospy.Subscriber("/car_state/complete", customMessage, car_state_cb)

	global graph
	graph = Graph()

	global uav_wp
	uav_wp = PoseStamped()

	global start_uav, start_ugv
	start_uav = Pose()
	start_ugv = Pose()

	global car_back, car_front, is_car
	car_back = None
	car_front = None
	is_car = None
	# start_uav = current_uav_pose.pose
	# start_ugv = start_uav
	# start_ugv.position.z = start_ugv.position.z - UAV_HEIGHT
	

	global end_ugv
	end_ugv = Pose()

	# , current_uav_xyz
	# joint_traj_arr = np.array([])
	# current_uav_xyz = np.array([])
	# print(joint_traj_arr.shape)

	# rate = rospy.Rate(10.0)
	# while not rospy.is_shutdown():
	# 	build_graph()
	# 	rate.sleep()

	rospy.spin()
		


if __name__ == "__main__":
	graph_planner()