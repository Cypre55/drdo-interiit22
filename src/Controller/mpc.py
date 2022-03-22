#!/usr/bin/env python
# from enum import Flag
from pyexpat import XML_PARAM_ENTITY_PARSING_ALWAYS
import rospy
from prius_msgs.msg import Control    
from nav_msgs.msg import Odometry 
from nav_msgs.msg import Path
import casadi as ca
import numpy as np    
import math   
from scipy.spatial import KDTree
from geometry_msgs.msg import Point,PoseStamped, Vector3
from tf.transformations import euler_from_quaternion
import copy 

from drdo_interiit22.msg import customMessage


from gazebo_msgs.msg import ModelStates	

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt

pi = math.pi
inf = np.inf

cross_track_error = []
path_to_store = []
"""# variable parameters 
"""
n_states = 4                                                                    
n_controls = 2

metres_ahead = 4.0


N =12#73                                                                           # Prediction horizon(same as control horizon)
error_allowed = 0.05
U_ref = np.array([0,0], dtype ='f')                                             # U_ref contains referance acc and steer
V_ref = 0.3#6#10                                                                      # referance velocity 



Q_x =41500                                                                      # gains to control error in x,y,V,theta during motion
Q_y = 41500 
Q_V = 10                                                                          
Q_theta = 5000

R1 = 201000
R2 = 55000


# Q_x =36500                                                                      # gains to control error in x,y,V,theta during motion
# Q_y = 36500 
# Q_V = 10                                                                          
# Q_theta = 1000

# R1 = 191000
# R2 = 55000

error_allowed_in_g = 1e-100                                                   # error in contraints
pos_drones = np.array([0.0,0.0,0.0])




"""# parameters that depend on simulator """


throttle_constant = 4.2                                                      # throttle_constant = acc/throttle
steer_constant = 40*pi/180                                                   # steer_constant = steer/steer_input
l_car = 3

n_bound_var = n_states              
x_bound_max = inf                                                            # enter x and y bounds when limited world like indoor environments                     
x_bound_min = -inf                     
y_bound_max = inf                      
y_bound_min = -inf 
V_bound_max = 8#inf                                                                           
V_bound_min = -8#-inf
acc_max  = 2.5            
theta_bound_max = inf                     
theta_bound_min = -inf  
pos_drones = np.array([0.0,0.0,0.0])                                                  # throttle_max = 1                                                                                                                                             
acc_min = -acc_max                                                                                                                                                                                                  
steer_max = 40*pi/180                                                        # steer_input_max = 1                                                   
steer_min = -steer_max



throttle,steer_input = None,None                                      # (x,y,V,theta) will store the current position,current speed and orientation 
																			 # throttle and steer_input will store the inputs to the vehicle 
x,y,V,theta,z = 0,0,0,0,0
x_prev,y_prev,V_prev, theta_prev = 0.0,0.0,0.0,0.0								 
total_path_points = None   
is_ninety = True
is_car = True                                                                                                                                
total_path_points = 0                                                                                                                                                                            
path = None                                                                                                                               

flag = 0
brake_vals = 0

def vfunc(odom_d):
	# print("Entered")
	
	global pos_drones
	
	pos_drones[0] = odom_d.pose.pose.position.x
	pos_drones[1] = odom_d.pose.pose.position.y
	pos_drones[2] = odom_d.pose.pose.position.z 


def equidist_path(path,total_path_points):

	# global total_path_points,path
	# global path
	
	resolution = 0.1
	t = np.linspace(0, total_path_points, total_path_points)/ 50

	smallS = 80
	
	factor = 10
	t_new = np.linspace(0, total_path_points*factor, total_path_points*factor)/ (50*factor)

	x,y = path[:,0],path[:,1]


	print (t.shape, x.shape)
	tck = interpolate.splrep(t, x, s = smallS, k = 5)
	x_new = interpolate.splev(t_new, tck, der=0)


	tck = interpolate.splrep(t, y, s = smallS, k = 5)
	y_new = interpolate.splev(t_new, tck, der=0)
	# print(len(path_x))
	# print(x_new.shape)
 




	#equidist


	distance = np.cumsum(np.sqrt( np.ediff1d(x_new, to_begin=0)**2 + np.ediff1d(y_new, to_begin=0)**2 ))

	n_points = int( distance[-1]/resolution )

	distance = distance/distance[-1]


	fx, fy = interp1d( distance, x_new ), interp1d( distance, y_new )

	alpha = np.linspace(0, 1, n_points)
	x_regular, y_regular = fx(alpha), fy(alpha)

	x_regular = np.expand_dims(x_regular, axis=-1)
	y_regular = np.expand_dims(y_regular, axis=-1)


	path_final = np.concatenate([x_regular, y_regular], axis = 1)
	plt.plot(path[:,0],path[:,1], color = 'r') 
	plt.plot(path_final[:,0],path_final[:,1], color = 'b')
	# path = path_final

	plt.show()
	return path_final
	

#def pathfunc(Path):                                                                                                                                       
def pathfunc():	
	
	global total_path_points,path
	global flag
	if flag == 0:
		flag = 1
		if total_path_points == 0:
			
			# total_path_points = len(Path.poses)
			# total_path_points = len(path_x)
			# path = np.load("/home/satwik/catkin_ws/src/drdo-interiit22/graph_nodes.npy")
			path = np.load("Full_World1_RUN.npy")
			path = path[250:]
			total_path_points = path.shape[0] #(path[:,0]).size

			# print("Path size before sampling : ",path.shape)

			path = equidist_path(path,total_path_points)

			# print("Path size after sampling : ",path.shape)

			# path = np.array([[i.UGV.positi] for i in path])
			
			#path = np.zeros((total_path_points,2))													

		

	# for i in range(0,total_path_points):                                                                                                                
		# path[i][0] = Path.poses[i].pose.position.x
		# path[i][1] = Path.poses[i].pose.position.y  
		# path[i][0] = path_x[i] 
		# path[i][1] = path_y[i]	   	

first = 0

def odomfunc(odom):
	
	global x,y,V,theta,is_ninety, is_car,x_prev,y_prev,V_prev,theta_prev,z
	global first

	x = odom.car_state.pose.pose.position.x
	y = odom.car_state.pose.pose.position.y
	z = odom.car_state.pose.pose.position.z
	is_ninety = odom.isCarNinety.data
	is_car = odom.isMaskDetected.data

	# theta_prev1 = theta

	quaternions =  odom.car_state.pose.pose.orientation

	V = math.sqrt(odom.car_state.twist.twist.linear.x**2 + odom.car_state.twist.twist.linear.y**2)

	quaternions_list = [quaternions.x,quaternions.y,quaternions.z,quaternions.w]
	roll,pitch,yaw = euler_from_quaternion(quaternions_list)
	theta = yaw

	

	if np.abs(x)<0.001:
		# print("PREV", x)
		x,y,V,theta = x_prev, y_prev,V_prev,theta_prev
		# print("AFTER",x)  
	else:
		x_prev,y_prev,V_prev,theta_prev = x,y,V, theta_prev  
	
	# if (np.abs(theta - theta_prev) > 0.3):
	# 	theta = theta_prev

  
	# if np.abs(V)<0.001:
	# 	# print("PREV", x)
	# 	V = V_prev  
	# 	# print("AFTER",x)  
	# else:
	# 	V_prev = V   
	# if np.abs(theta)<0.001:
	# 	# print("PREV", x)
	# 	theta = theta_prev  
	# 	# print("AFTER",x)  
	# else:
	# 	theta_prev = theta

	# if np.abs(theta_prev-theta)





delta_z = None
slope_throttle = 1

# def lane_norm_cb(data):
# 	global delta_z
# 	delta_z = data.z
flag_drone_wait = 0

def my_mainfunc():
	
	global is_ninety, is_car, x_prev,y_prev,V_prev,x,y,V,brake_vals,pos_drones,flag_drone_wait,z
	global path
	rospy.init_node('mpc_multipleShooting_pathTracking_carDemo', anonymous=True)
	# rospy.Subscriber('/base_pose_ground_truth' , Odometry, odomfunc)   
	# rospy.Subscriber('/mavros/local_position/odom' , Odometry, odomfunc)  
	# rospy.Subscriber('/gazebo/model_states' , ModelStates, odomfunc)    
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)    
	# rospy.Subscriber('/car_state/kalman_complete' , customMessage, odomfunc)
	# rospy.Subscriber('/car_state/filter_complete' , customMessage, odomfunc)    
	
	# rospy.Subscriber('/lane/norm',Vector3,lane_norm_cb)   
	rospy.Subscriber('/mavros/local_position/odom' , Odometry, vfunc) 


	# path = np.load("Full_World1_RUN.npy")
	path = np.load("path_new_rs.npy").T[20:,0:2]
	

	total_path_points = (path[:,0]).size
	# print("Path siez before sampling : ",path.shape)

	path = equidist_path(path,total_path_points)
	# print("Last path point : ",path[-1])
	# print("Path siez after sampling : ",path.shape)

	total_path_points = (path[:,0]).size
	 
	print("Total", path.shape)
	#rospy.Subscriber('/astroid_path ', Path, pathfunc)
	# pathfunc()

	instance = rospy.Publisher('prius', Control, queue_size=10)
	pub1 = rospy.Publisher("uav_wp", PoseStamped, queue_size=1000000) 
	rate = rospy.Rate(10)
	rate.sleep()                                                                                 #rate.sleep() to run odomfunc once 

	msg = Control()
	drone_msg = PoseStamped()
	path_resolution =  ca.norm_2(path[0,0:2] - path[1,0:2])                                                                                        
	global delta_T                                                                               #timestamp bw two predictions                                                                                                                  
	delta_T = ( path_resolution / ((V_ref)) )/10                                                                                                                                                                                                           
	
	
	
	"""MPC"""

	x_casadi =ca.SX.sym('x')
	y_casadi = ca.SX.sym('y')
	V_casadi = ca.SX.sym('V')
	theta_casadi = ca.SX.sym('theta')
	states =np.array([(x_casadi),(y_casadi),(V_casadi),(theta_casadi)])
	n_states = states.size   

	acc_casadi =ca.SX.sym('acc')
	steer_casadi = ca.SX.sym('steer')
	controls = np.array([acc_casadi,steer_casadi])
	n_controls = controls.size  

	rhs = np.array([V_casadi*ca.cos(theta_casadi),V_casadi*ca.sin(theta_casadi),acc_casadi,ca.tan(steer_casadi)/l_car])                                              
	f = ca.Function('f',[states,controls],[rhs])                                                                            # function to predict rhs using states and controls        

	U = ca.SX.sym('U', n_controls,N)                                                                                        # For storing predicted controls  
	X =ca.SX.sym('X', n_states, N+1)                                                                                        # For storing predicted states 
	P = ca.SX.sym('P',1, n_states + n_states*(N) + n_controls*(N) )                                                         # For storing odometry, next N path points and next N referance controls                                                  

	
	
	obj = 0
	g = []

	Q = ca.diagcat(Q_x, Q_y,Q_V, Q_theta)   
	R = ca.diagcat(R1, R2) 

	for i in range(0,N):                                                                                                                                                                                                                         
		cost_pred_st = ca.mtimes(  ca.mtimes( (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) ).T , Q )  ,  (X[0:n_states,i] - P[n_states*(i+1) :n_states*(i+1) + n_states ].reshape((n_states,1)) )  )  + ca.mtimes(  ca.mtimes( ( (U[0:n_controls,i]) - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1)) ).T , R )  ,  U[0:n_controls,i] - P[n_states*(N+1)+n_controls*(i):n_states*(N+1)+n_controls*(i) + n_controls].reshape((n_controls,1))  )  
		obj = obj + cost_pred_st  
	pred_st = np.zeros((n_states,1))     
	for i in range(0,N+1):                                                                                                   # adding contraints so the predictions are in sync with vehicle model  
		if i == 0:
			g = ca.vertcat( g,( X[0:n_states,i] - P[0:n_states].reshape((n_states,1)) )  )                                                             
		else:
			# f_value = f(X[0:n_states,TP_befi-1],U[0:n_controls,i-1])                                                             # euler method not used 
			# pred_st = X[0:n_states,i-1] + delta_T*f_value                                                                    

			K1 = f(X[0:n_states,i-1],U[0:n_controls,i-1])                                                                    # Runge Kutta method of order 4 
			K2 = f(X[0:n_states,i-1] + np.multiply(K1,delta_T/2),U[0:n_controls,i-1])                                              
			K3 = f(X[0:n_states,i-1] + np.multiply(K2,delta_T/2),U[0:n_controls,i-1])                                           
			K4 = f(X[0:n_states,i-1] + np.multiply(K3,delta_T),U[0:n_controls,i-1])                                                  
			pred_st = X[0:n_states,i-1] + (delta_T/6)*(K1+2*K2+2*K3+K4)                                                      # predicted state                                              
			 
			g = ca.vertcat( g,(X[0:n_states,i] - pred_st[0:n_states].reshape((n_states,1)) )  )                      
 
	OPT_variables = X.reshape((n_states*(N+1),1))          
	OPT_variables = ca.vertcat( OPT_variables, U.reshape((n_controls*N,1)) )     

	nlp_prob ={
			'f':obj,
			'x':OPT_variables,
			'g':g,
			'p':P
			}
	
	opts = {
			 'ipopt':
			{
			  'max_iter': 100,
			  'print_level': 0,
			  'acceptable_tol': 1e-8,
			  'acceptable_obj_change_tol': 1e-6
			},
			 'print_time': 0
		   }
	
	
	solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
	
	
	
	lbg = ca.DM.zeros(((n_states)*(N+1),1))                                                                           # bounds on g                                                                                                           
	ubg = ca.DM.zeros(((n_states)*(N+1),1))                              

	lbg[0:(n_states)*(N+1)] = - error_allowed_in_g                                                                                                  
	ubg[0:(n_states)*(N+1)] =  error_allowed_in_g                                                                                                    
	
	lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1))                                                              # bounds on X  
	ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N,1)) 

	lbx[0:n_bound_var*(N+1):n_states] = x_bound_min                     
	ubx[0:n_bound_var*(N+1):n_states] = x_bound_max                     
	lbx[1:n_bound_var*(N+1):n_states] = y_bound_min                     
	ubx[1:n_bound_var*(N+1):n_states] = y_bound_max                     
	lbx[2:n_bound_var*(N+1):n_states] = V_bound_min                 
	ubx[2:n_bound_var*(N+1):n_states] = V_bound_max 
	lbx[3:n_bound_var*(N+1):n_states] = theta_bound_min                 
	ubx[3:n_bound_var*(N+1):n_states] = theta_bound_max                

	lbx[n_bound_var*(N+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = acc_min                       
	ubx[(n_bound_var*(N+1)):(n_bound_var*(N+1)+n_controls*N):n_controls] = acc_max                     
	lbx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = steer_min               
	ubx[(n_bound_var*(N+1)+1):(n_bound_var*(N+1)+n_controls*N):n_controls] = steer_max  



	X_init = np.array([x,y,V,theta], dtype = 'f')                                                                                                               
	X_target = np.array([ path[total_path_points-1][0], path[total_path_points-1][1],0, 0 ]  , dtype = 'f')            #theta_target not considered for stopping condition and Velocity at target is zero !  

	P = X_init   
	P_copy = copy.deepcopy(P)   
	close_index = KDTree(path).query(P_copy[0:n_states-2])[1]

	for i in range(0,N):                                                                              
		P = ca.vertcat(P,path[close_index+i,0:2])        
		P = ca.vertcat(P,V_ref)                                                                                                                                                                
		P = ca.vertcat(P, math.atan((path[close_index+i+1][1] - path[close_index+i][1])/(path[close_index+i+1][0] - path[close_index+i][0])) )    
		
	for i in range(0,N):                                                                              
		P = ca.vertcat(P, U_ref[0])                                                                   
		P = ca.vertcat(P, U_ref[1])    

	initial_X = ca.DM.zeros((n_states*(N+1)))                                                                           #all initial search values of predicted states are X_init
	initial_X[0:n_states*(N+1):n_states] = X_init[0]
	initial_X[1:n_states*(N+1):n_states] = X_init[1]
	initial_X[2:n_states*(N+1):n_states] = X_init[2]
	initial_X[3:n_states*(N+1):n_states] = X_init[3]      

	initial_con = ca.DM.zeros((n_controls*N,1))                                                                         #initial search values of control matrix are zero

	run_drn_flag = True
	run_car_flag = False
	drn_ref_pt_idx = 1

	

	try:
		while ( ca.norm_2( P[0:n_states-1].reshape((n_states-1,1)) - X_target[0:n_states-1] ) > error_allowed  ) :       
			# if np.abs(x)<0.001:
			# 	# print("PREV", x)
			# 	x,y,V = x_prev, y_prev,V_prev  
			# 	# print("AFTER",x)  
			# else:
			# 	x_prev,y_prev,V_prev = x,y,V       
			# if np.abs(V)<0.001:
			# 	# print("PREV", x)
			# 	V = V_prev  
			# 	# print("AFTER",x)  
			# else:
			# 	V_prev = V                                                                            
			args = {
					'lbx':lbx,
					'lbg':lbg,	    
					'ubx':ubx,
					'ubg':ubg,
					'p':P,
					'x0':ca.vertcat(initial_X,initial_con),                                      
				}
			
			sol = solver(
							
						x0=args['x0'],
						
						lbx=args['lbx'],
						ubx=args['ubx'],
						
						lbg=args['lbg'],
						ubg=args['ubg'],
						p=args['p']
							
						)       
		
			X_U_sol = sol['x']

			acc = (X_U_sol[n_states*(N+1)].full())[0][0]      
			steer = (X_U_sol[n_states*(N+1)+1].full())[0][0]

			throttle = acc / throttle_constant
			steer_input = steer/steer_constant

			msg.throttle = throttle                                  
			msg.brake = 0.0 
			msg.steer = steer_input
			msg.shift_gears =2

			if throttle < 0:
				# msg.shift_gears =3                                              # reverse gear
				# throttle = -throttle
				msg.throttle = 0.0                                  
				msg.brake = -throttle  												#brake


			# 	msg.steer = steer_input
			# 	msg.shift_gears =2
			
			# if delta_z 
			if V>4.5:
				msg.brake  = 0.35

			# if not (is_ninety):
			# 	# brake_vals +=1
			# 	# if brake_vals ==4:
			# 	print("90_perc brake")
			# 	msg.brake = 0.4
			# 		# brake_vals = 0
			# print("Path size before kdtree : ",path.shape)
			
			# print("Path size after kdtree : ",path.shape)

			
			# if (index_drone < close_index) or  (np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)<3.7):
			# 	flag_drone_wait = 1
			# 	# print("Waiting for the drone")
			# 	# while (index_drone < close_index or np.abs((np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2))-4)<0.5):
			# 	# 	if index_drone < close_index:
			# 	# 		print("CHECK 1")

			# 	# 	if (np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)>4):
			# 	# 		print("check 2")
			# 	# 	print("Waiting for the drone in whie")
			# 	# 	msg.brake = 1
			# 	# 	msg.throttle = 0
			# 	# 	rate.sleep()
			# 	# 	instance.publish(msg)
			# 		# rospy.spin()
			# if flag_drone_wait == 1:
			# 	print("waiting for the drone")
			# 	msg.brake = 1
			# 	msg.throttle = 0
			# 	# instance.publish(msg)
			# 	boundary = 0.2
				
			# 	condition1 = (index_drone>close_index) and np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)<(4.0 +boundary)
			# 	condition2 = np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)<(4.0 - boundary)
				
			# 	if condition1 or condition2:
			# 		if (index_drone>close_index):
			# 			print("DUE TO INDEX")
			# 		if (np.abs(np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)-4.0)<0.2):
			# 			print("due to pos")
			# 		print("Turning off flag")
			# 		flag_drone_wait = 0

			"""
			global metres_ahead
			if (index_drone < close_index) and (np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)>(metres_ahead-0.4)):
				flag_drone_wait = 1
				# print("Waiting for the drone")
				# while (index_drone < close_index or np.abs((np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2))-4)<0.5):
				# 	if index_drone < close_index:
				# 		print("CHECK 1")

				# 	if (np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)>4):
				# 		print("check 2")
				# 	print("Waiting for the drone in whie")
				# 	msg.brake = 1
				# 	msg.throttle = 0
				# 	rate.sleep()
				# 	instance.publish(msg)
					# rospy.spin()
			if flag_drone_wait == 1:
				print("waiting for the drone")
				msg.brake = 1
				msg.throttle = 0
				# instance.publish(msg)
				if (index_drone>close_index) and (np.abs(np.sqrt((x-pos_drones[0])**2+(y-pos_drones[1])**2)-metres_ahead)<0.3):
					print("Turning off flag")
					flag_drone_wait = 0
			"""

			#MOVE DRONE
			close_index = KDTree(path).query(np.array([x,y]))[1]
			index_drone = KDTree(path).query(np.array([pos_drones[0],pos_drones[1]]))[1]
			# print("UGV Pose",(x,y))
			# print("UAV Pose",(pos_drones[0],pos_drones[1]))
			# print("Drone_index", index_drone)
			# print("Car_index", close_index)
			index_or_condition = ((index_drone-close_index)<10)
			index_ahead = 45
			drone_height = 18
			
			if ((index_drone<=close_index) or index_or_condition) and not (is_ninety):
				print("MOVING DRONE")

				drone_msg.pose.position.x,drone_msg.pose.position.y,drone_msg.pose.position.z = (path[int(close_index+index_ahead)][0]), (path[int(close_index+index_ahead)][1]),drone_height+z #TILL CAR
				print("drone moving to",drone_msg.pose.position.x,drone_msg.pose.position.y,drone_msg.pose.position.z)
			if ((index_drone<=close_index) or index_or_condition) and not (is_ninety):
				print("-------------------")
				print("WAITING for drone")
				msg.brake = 1
				msg.throttle = 0
			
			else:
				msg.brake = 0

			if ((np.sqrt((pos_drones[0] - drone_msg.pose.position.x)**2+(pos_drones[1] - drone_msg.pose.position.y)**2)) > 2 ) and (np.abs(drone_msg.pose.position.x)>0):

				print("-------------------")
				print("WAITING for drone mk2")
				print("drone moving to",drone_msg.pose.position.x,drone_msg.pose.position.y,drone_msg.pose.position.z)
				print("donre position",pos_drones[0],pos_drones[1],pos_drones[2])
				pub1.publish(drone_msg)
				 
				# drone_msg.pose.position.x,drone_msg.pose.position.y,drone_msg.pose.position.z = (path[int(close_index+index_ahead)][0]), (path[int(close_index+index_ahead)][1]),drone_height+z #TILL CAR

				msg.brake = 1
				msg.throttle = 0

			#####
			# if not run_car_flag:
			# 	print("Running drone")
			# 	msg.brake = 1
			# 	msg.throttle = 0

			# 	factor_ = 10
			# 	print("drn_ref_pt_idx", drn_ref_pt_idx)
				
			# 	if (is_car and not is_ninety) and (index_drone<close_index) and run_drn_flag:
			# 		print("Running drone when car in boundary and car ahead")
					
					
			# 		drone_msg.pose.position.x = path[drn_ref_pt_idx*factor_][0]
			# 		drone_msg.pose.position.y = path[drn_ref_pt_idx*factor_][1]
			# 		drone_msg.pose.position.z = drone_height+z
			# 		## while publish drone pts iterative
			# 		if (np.sqrt((pos_drones[0] - drone_msg.pose.position.x)**2+(pos_drones[1] - drone_msg.pose.position.y)**2)) > 1.5 :
			# 			print("Going to the new index")
						
			# 		else:
			# 			drn_ref_pt_idx +=1
			# 			print("Increasing index")

			# 		run_drn_flag = True

			# 	if (is_car and is_ninety) or run_drn_flag:
			# 		print("Running drone when car in boundary and car center")

			# 		drone_msg.pose.position.x = path[drn_ref_pt_idx*factor_][0]
			# 		drone_msg.pose.position.y = path[drn_ref_pt_idx*factor_][1]
			# 		drone_msg.pose.position.z = drone_height+z

			# 		## while publish drone pts iterative
			# 		if (np.sqrt((pos_drones[0] - drone_msg.pose.position.x)**2+(pos_drones[1] - drone_msg.pose.position.y)**2)) > 1.5:
			# 			print("Going to the new index")
						
			# 		else:
			# 			drn_ref_pt_idx +=1
			# 			print("Increasing index")

			# 		run_drn_flag = True

			# 	# print("c1", is_car , (not is_ninety))
			# 	# print("c2",run_drn_flag)
			# 	# print("c3",index_drone,close_index)
			# 	if (not is_ninety) and run_drn_flag and (index_drone>close_index):
			# 		print("Stopping drone")

			# 		run_drn_flag = False
			# 		run_car_flag = True

			# if not run_drn_flag:
			# 	print("Running car")
			# 	if run_car_flag:
					
			# 		print("Running car mpc")
			# 		# throttle = acc / throttle_constant
			# 		# steer_input = steer/steer_constant

			# 		# msg.throttle = throttle                                  
			# 		# msg.brake = 0.0 
			# 		# msg.steer = steer_input
			# 		# msg.shift_gears =2

			# 		# if throttle < 0:
			# 		# 	# msg.shift_gears =3                                              # reverse gear
			# 		# 	# throttle = -throttle
			# 		# 	msg.throttle = 0.0                                  
			# 		# 	msg.brake = -throttle  		

			# 		## run mpc

			# 		run_car_flag = True

			# 	if ( not is_ninety) and close_index>index_drone:
			# 		print("Stopping car")
			# 		msg.brake = 1
			# 		run_car_flag = False
			# 		run_drn_flag = True

			#####
			

			
			# if (index_drone>close_index) and not (is_ninety):



			# print("THROTLE", msg.throttle, "BRAKE", msg.brake)
			# print("THETA", theta,x,y)
			print("CAR INDEX", close_index)
			print("Total", path.shape)
			pub1.publish(drone_msg)
			instance.publish(msg)
			#print ('   Velocity (in m/s)  = ',round(V,2))
			# print(x,y,V,theta)

			x_copy = copy.deepcopy(x)
			y_copy = copy.deepcopy(y)
			# print("x,y before : ",x,y)
			path_to_store.append([x,y])
			# cross_track_error.append(KDTree(path).query(np.array([x_copy,y_copy]))[0])
			# print("x,y after : ",x,y)

			P[0:n_states] = [x,y,V,theta] 
			# print("Yaw : ",theta)
			arrow_size = 40
			# plt.clf()

			# plt.Circle((x,y),(metres_ahead-0.3))
			# plt.Circle((x,y),(metres_ahead+0.3))
			# plt.scatter(pos_drones[0],pos_drones[1])
			# plt.axes.set_aspect("equal")

			# fig, ax = plt.subplots()
			# ax.add_patch(c1)
			# ax.add_patch(c2)

			# plt.xlim(-100,400)
			# plt.ylim(-100,400)
			# plt.plot(path[:,0],path[:,1])
			# plt.arrow(x,y,x + arrow_size*math.cos(theta),y+arrow_size*math.sin(theta), head_width = 7, head_length = 7)
			# plt.pause(0.0001)
			if (N+close_index) < total_path_points :  
				# print("X and Y before", x,y)
				# print("CI Before", close_index)
				# print("TP_before", total_path_points)                                                                              # Updating P for next N path points and next N reference controls
				P[n_states:n_states*(N+1):n_states] = path[close_index:N+close_index,0] 
				P[n_states+1:n_states*(N+1):n_states] = path[close_index:N+close_index,1]
				for i in range(0,N):                                                                
					P[n_states*(i+1+1)-2] = V_ref                                                                                                                          
					P[n_states*(i+1+1)-1] = math.atan( (path[i+close_index+1][1] - path[i+close_index][1])/(path[i+close_index+1][0] - path[i+close_index][0] + 1e-9) )  

				P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                             
				P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref   

			else:
				# print("X and Y after", x,y)
				# print("N",N, "ci", close_index)
				# print("TP", total_path_points)
				# print ("The end point in inside horizon, slowing down")
				P[n_states:n_states*(N)] = P[n_states*2:n_states*(N+1)]                                                                  
				P[n_states*(N):n_states*(N+1)-2] = path[(total_path_points-1),0:2]                                                                                                                                                                                                                                                                                                           
				P[n_states*(N+1)-2] = 0 #V_ref                                                                    #we need to stop the bot at end, hence referance velocity 0 at end                                                                                                                              #
				P[n_states*(N+1)-1] = math.atan( (path[total_path_points-1][1] - path[total_path_points-1-1][1])/(path[total_path_points-1][0] - path[total_path_points-1-1][0]) )
					
				P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                                  
				P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref       
				
			for i in range(0,N*n_states):                              
				initial_X[i] = X_U_sol[i+n_states]                                                                #initial search value of state for next iteration should be the predicted one for that iteration         

			for i in range(0,(N-1)*n_controls):                      
				initial_con[i] = X_U_sol[n_states*(N+1)+i+n_controls]                                             #initial search value of control for next iteration should be the predicted one for that iteration

			rate.sleep()
		plt.show()

	except KeyboardInterrupt:

		print("saving")
		cte = np.array(path_to_store)
		np.save("path_followed", cte)

	# print ("PATH TRACKED")
	msg.throttle = 0                                                                    # stopping the vehicle                       
	msg.brake = 1 
	msg.steer = 0
	msg.shift_gears =1                                                                  # nuetral gear    
	instance.publish(msg)

	  


if __name__ == '__main__':
   
	try:
		my_mainfunc()
	except rospy.ROSInterruptException:
		pass
