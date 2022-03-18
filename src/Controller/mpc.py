#!/usr/bin/env python
# from drdo_interiit22.src.Controller.kalman_dummy import R
import rospy
from prius_msgs.msg import Control    
from nav_msgs.msg import Odometry 
from nav_msgs.msg import Path
import casadi as ca
import numpy as np    
import math   
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion 

from drdo_interiit22.msg import customMessage


from gazebo_msgs.msg import ModelStates	

import scipy.interpolate as interpolate
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt

pi = math.pi
inf = np.inf

cross_track_error = []




"""# variable parameters 
"""
n_states = 4                                                                    
n_controls = 2



N =67#73                                                                           # Prediction horizon(same as control horizon)
error_allowed = 0.1
U_ref = np.array([0,0], dtype ='f')                                             # U_ref contains referance acc and steer
V_ref = 0.3#6#10                                                                      # referance velocity 


Q_x = 230000#3000                                                                      # gains to control error in x,y,V,theta during motion
Q_y = 230000#3000 
Q_V = 10#1000000                                                                          
Q_theta = 1000#200 

R1 = 1e+14	#0.5*1e+5#8#1e+15#100000                                                                     # gains to control acc and steer                                                                                                           
R2 = 1e+7#10000

error_allowed_in_g = 1e-100                                                   # error in contraints





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
theta_bound_max = inf                     
theta_bound_min = -inf  

acc_max = 4.2                                                                # throttle_max = 1                                                                                                                                             
acc_min = -acc_max                                                                                                                                                                                                  
steer_max = 40*pi/180                                                        # steer_input_max = 1                                                   
steer_min = -steer_max



global x,y,V,theta,throttle,steer_input                                      # (x,y,V,theta) will store the current position,current speed and orientation 
																			 # throttle and steer_input will store the inputs to the vehicle 
																			 
global total_path_points                                                                                                                                    
total_path_points = 0                                                                                                                                                                            
global path                                                                                                                                       

flag = 0

pose_for_quiver_x = []
pose_for_quiver_y = []
pose_for_quiver_u = []
pose_for_quiver_v = []

kalman_pose_for_quiver_x = []
kalman_pose_for_quiver_y = []
kalman_pose_for_quiver_u = []
kalman_pose_for_quiver_v = []

def equidist_path(path,total_path_points):

	# global total_path_points,path
	
	resolution = 0.1
	t = np.linspace(0, total_path_points, total_path_points)/ 50

	smallS = 10000
	# print(t)
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
	# plt.plot(path[:,0],path[:,1], color = 'r') 
	# plt.plot(path_final[:,0],path_final[:,1], color = 'b')
	path = path_final

	# plt.show()
	return path
	

#def pathfunc(Path):                                                                                                                                       
def pathfunc():	
	
	global flag
	if flag == 0:
		flag = 1
		global total_path_points,path
		if total_path_points == 0:
			
			# total_path_points = len(Path.poses)
			# total_path_points = len(path_x)
			# path = np.load("/home/satwik/catkin_ws/src/drdo-interiit22/graph_nodes.npy")
			path = np.load("/home/rohit_dhamija/InterIIT22_ws/src/drdo_interiit22/src/Controller/ugv_waypoints.npy")
			total_path_points = (path[:,0]).size

			path = equidist_path(path,total_path_points)

			# path = np.array([[i.UGV.positi] for i in path])
			
			#path = np.zeros((total_path_points,2))													

		

	# for i in range(0,total_path_points):                                                                                                                
		# path[i][0] = Path.poses[i].kalman_.position.x
		# path[i][1] = Path.poses[i].kalman_.position.y  
		# path[i][0] = path_x[i] kalman_
		# path[i][1] = path_y[i]	   	

Xnp = np.zeros((6, 1), dtype = np.float32)


dt = 0.1 #
F = np.array(  [[1, 0, dt,  0, 0.5*dt*dt,         0],
				[0, 1,  0, dt,         0, 0.5*dt*dt],
				[0, 0,  1,  0,        dt,         0],
				[0, 0,  0,  1,         0,        dt],
				[0, 0,  0,  0,         1,         0],
				[0, 0,  0,  0,         0,         1]], dtype=np.float32) #transition matrix
u = np.zeros((6, 1), dtype=np.float32) #transition offset
B = np.array([[1, 0, 0, 0, 0, 0],
			  [0, 1, 0, 0, 0, 0],
			  [0, 0, 1, 0, 0, 0],
			  [0, 0, 0, 1, 0, 0],
			  [0, 0, 0, 0, 1, 0],
			  [0, 0, 0, 0, 0, 1]], dtype=np.float32)

H = np.array([[1, 0, 0, 0, 0, 0],
				[0, 1, 0, 0, 0, 0],
				[0, 0, 1, 0, 0, 0],
				[0, 0, 0, 1, 0, 0]], dtype=np.float32)
P = np.array(  [[1, 0, 0, 0, 0, 0],
				[0, 1, 0, 0, 0, 0],
				[0, 0, 1, 0, 0, 0],
				[0, 0, 0, 1, 0, 0],
				[0, 0, 0, 0, 1, 0],
				[0, 0, 0, 0, 0, 1]], dtype=np.float32)
R = np.array(  [[100, 0, 0, 0],
				[0, 100, 0, 0],
				[0, 0, 100, 0],
				[0, 0, 0, 100]], dtype=np.float32) # measurement uncertainty

Q = np.array(  [[0.5, 0, 0, 0, 0, 0],
				[0, 0.5, 0, 0, 0, 0],
				[0, 0, 0.1, 0, 0, 0],
				[0, 0, 0, 0.1, 0, 0],
				[0, 0, 0, 0, 1, 0],
				[0, 0, 0, 0, 0, 1]], dtype=np.float32) # estimate uncertainty
I = np.identity(6, dtype=np.float32)

# Mark 1
# Q *= 0.1
# R *= 100

# Mark 2
# R = np.array(  [[100, 0, 0, 0],
# 				[0, 100, 0, 0],
# 				[0, 0, 100, 0],
# 				[0, 0, 0, 100]], dtype=np.float32) # measurement uncertainty

# Q = np.array(  [[0.5, 0, 0, 0, 0, 0],
# 				[0, 0.5, 0, 0, 0, 0],
# 				[0, 0, 0.1, 0, 0, 0],
# 				[0, 0, 0, 0.1, 0, 0],
# 				[0, 0, 0, 0, 1, 0],
# 				[0, 0, 0, 0, 0, 1]], dtype=np.float32) # estimate uncertainty
# Q *= 1
# R *= 10

Q *= 1
R *= 10

def kalman(z):
	global Xnp, F, u, H, P, R, I, B, Q

	z = np.array(z)
	z = np.expand_dims(z, axis=1)
	y = z - np.matmul(H,Xnp)
	S = np.matmul(H,np.matmul(P,H.T)) + R
	Sinv = np.linalg.inv(S)
	k = np.matmul(P,(np.matmul((H.T),Sinv)))
	Xnp = Xnp + np.matmul(k,y)
	P = np.matmul((I - np.matmul(k,H)),P)

	Xnp = np.matmul(F,Xnp) + np.matmul(B, u)
	P = np.matmul(F,np.matmul(P,F.T)) + Q

first = 0

def odomfunc(odom):

	global x,y,V,theta, Xnp
	global pose_for_quiver_x
	global pose_for_quiver_y
	global pose_for_quiver_u
	global pose_for_quiver_v
	global kalman_pose_for_quiver_x
	global kalman_pose_for_quiver_y
	global kalman_pose_for_quiver_u
	global kalman_pose_for_quiver_v

	x = odom.car_state.pose.pose.position.x
	y = odom.car_state.pose.pose.position.y

	global first

	if(first == 0):
		first = 1
		Xnp[0] = x
		Xnp[1] = y


	quaternions =  odom.car_state.pose.pose.orientation

	V = math.sqrt(odom.car_state.twist.twist.linear.x**2 + odom.car_state.twist.twist.linear.y**2)

	curr_pose = [x, y, odom.car_state.twist.twist.linear.x, odom.car_state.twist.twist.linear.y]

	kalman(curr_pose)

	pose_for_quiver_x.append(curr_pose[0])
	pose_for_quiver_y.append(curr_pose[1])
	pose_for_quiver_u.append(curr_pose[2])
	pose_for_quiver_v.append(curr_pose[3])
	kalman_pose_for_quiver_x.append(Xnp[0])
	kalman_pose_for_quiver_y.append(Xnp[1])
	kalman_pose_for_quiver_u.append(Xnp[2])
	kalman_pose_for_quiver_v.append(Xnp[3])

	x = np.squeeze(kalman_pose_for_quiver_x[-1])
	y = np.squeeze(kalman_pose_for_quiver_y[-1])
	V = math.sqrt(kalman_pose_for_quiver_u[-1]**2 + kalman_pose_for_quiver_v[-1]**2)

	quaternions_list = [quaternions.x,quaternions.y,quaternions.z,quaternions.w]
	roll,pitch,yaw = euler_from_quaternion(quaternions_list)
	theta = yaw



def my_mainfunc():
	rospy.init_node('mpc_multipleShooting_pathTracking_carDemo', anonymous=True)
	# rospy.Subscriber('/base_pose_ground_truth' , Odometry, odomfunc)   
	# rospy.Subscriber('/mavros/local_position/odom' , Odometry, odomfunc)  
	# rospy.Subscriber('/gazebo/model_states' , ModelStates, odomfunc)    
	rospy.Subscriber('/car_state/complete' , customMessage, odomfunc)    

	path = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/ugv_waypoints.npy")
	total_path_points = (path[:,0]).size

	path = equidist_path(path,total_path_points)
	# path_new = path[::10]
	# print("path shape,",path_new.shape)
     

	#rospy.Subscriber('/astroid_path', Path, pathfunc)
	# pathfunc()

	instance = rospy.Publisher('prius', Control, queue_size=10)

	rate = rospy.Rate(10)
	rate.sleep()                                                                                 #rate.sleep() to run odomfunc once 

	msg = Control()

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
			# f_value = f(X[0:n_states,i-1],U[0:n_controls,i-1])                                                             # euler method not used 
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
	close_index = KDTree(path).query(P[0:n_states-2])[1]

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

	x_hist = []
	y_hist= []

	

	try:
		while ( ca.norm_2( P[0:n_states-1].reshape((n_states-1,1)) - X_target[0:n_states-1] ) > error_allowed  ) :                                                                                         
			
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

			# plt.clf()
			# plt.plot(path[:800,0],path[:800,1])

			x_hist.append(x)
			y_hist.append(y)

			# print("path shape,",path.shape)

			global pose_for_quiver_x, pose_for_quiver_y, pose_for_quiver_u, pose_for_quiver_v, Xnp
			global kalman_pose_for_quiver_x, kalman_pose_for_quiver_y, kalman_pose_for_quiver_u, kalman_pose_for_quiver_v
			
			plt.clf()
			plt.plot(pose_for_quiver_x, pose_for_quiver_y, color='r')
			print('orig: ', len(pose_for_quiver_x), len(pose_for_quiver_y))
			try:
				plt.plot(kalman_pose_for_quiver_x, kalman_pose_for_quiver_y, color='b')
			except Exception as e:
				print('kalman: ', len(kalman_pose_for_quiver_x), len(kalman_pose_for_quiver_y))
				if(len(kalman_pose_for_quiver_x) > len(kalman_pose_for_quiver_y)):
					kalman_pose_for_quiver_x = kalman_pose_for_quiver_x[:-1]
				else:
					kalman_pose_for_quiver_y = kalman_pose_for_quiver_y[:-1]
			# plt.xlim(0, 50)
			# plt.ylim(-50, 0)
			# plt.quiver(kalman_pose_for_quiver_x,kalman_pose_for_quiver_y,kalman_pose_for_quiver_u,kalman_pose_for_quiver_v)
			plt.pause(0.0001)
			
			if throttle>0.01:
				throttle = 0.01   
			msg.throttle = throttle                              
			msg.brake = 0.0 
			msg.steer = steer_input
			# msg.shift_gears =2
			if throttle < 0:
				# msg.shift_gears =3                                              # reverse gear
				# throttle = -throttle
				msg.throttle = 0.0                                  
				msg.brake = -80*throttle												#brake
				msg.steer = steer_input
				# msg.shift_gears =2
			
			if V >1.5:
				msg.brake = 0.35
			instance.publish(msg)
			#print ('   Velocity (in m/s)  = ',round(V,2))
			print("Throttle", msg.throttle )
			cross_track_error.append(KDTree(path).query(np.array([x,y]))[0])


			P[0:n_states] = [x,y,V,theta] 

			close_index = KDTree(path).query(np.array([x,y]))[1]

			if N+(close_index) < total_path_points :                                                                                # Updating P for next N path points and next N reference controls
				P[n_states:n_states*(N+1):n_states] = path[close_index:N+close_index,0] 
				P[n_states+1:n_states*(N+1):n_states] = path[close_index:N+close_index,1]
				for i in range(0,N):                                                                
					P[n_states*(i+1+1)-2] = V_ref                                                                                                                          
					P[n_states*(i+1+1)-1] = math.atan( (path[i+close_index+1][1] - path[i+close_index][1])/(path[i+close_index+1][0] - path[i+close_index][0] + 1e-9) )  

				P[n_states*(N+1):n_states*(N+1)+n_controls*(N-1)]= P[n_states*(N+1)+n_controls:n_states*(N+1)+n_controls*(N)]                                                                                                                                                                                                             
				P[n_states*(N+1)+n_controls*(N-1):n_states*(N+1)+n_controls*(N)] = U_ref   

			else:
				
				print ("The end point in inside horizon, slowing down")
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
		cte = np.array(cross_track_error)
		np.save("cross_track_error_world1", cte)

	print ("PATH TRACKED")
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
