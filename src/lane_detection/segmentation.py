#!/usr/bin/python
from cv2 import transform
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool,Float64MultiArray
from geometry_msgs.msg import Point,PoseStamped, Vector3
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from skimage.morphology import binary_erosion,disk,skeletonize,binary_dilation
from scipy import ndimage as ndi
from scipy import interpolate
import time
import tf
from tf.transformations import quaternion_matrix
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
# from src.projection2Dto3D import projection

pre_est=np.array([0.,0.,1.]).reshape((1,1,3))
iscar=False
cx=-1
cy=-1
dep=np.zeros((480,640))
mask=np.zeros((480,640))
center = None
lane_mask=np.zeros((480,640))
drone_pose=PoseStamped()
road_norm=None
def fit_spline(x,y):
    resolution = 0.1
    path_x=[]
    path_y=[]
    xx=int(len(x)/50)
    xx=max(1,xx)
    for i in range(0,len(x),xx):
        path_x.append(x[i])
        path_y.append(y[i])
    t = np.linspace(0, len(path_x), len(path_x))/ 50
    smallS = 1000
    factor = 1
    x = np.array(path_x)
    try:
        tck = interpolate.splrep(t, x, s = smallS, k = 3)
        x_new = interpolate.splev(t, tck, der=0)
        y = np.array(path_y)
        tck = interpolate.splrep(t, y, s = smallS, k = 3)
        y_new = interpolate.splev(t, tck, der=0)
        distance = np.cumsum(np.sqrt( np.ediff1d(x_new, to_begin=0)**2 + np.ediff1d(y_new, to_begin=0)**2 ))
        n_points=50
        distance = distance/distance[-1]
        fx, fy = interpolate.interp1d( distance, x_new ), interpolate.interp1d( distance, y_new )
        alpha = np.linspace(0, 1, n_points)
        x_regular, y_regular = fx(alpha), fy(alpha)
        x_regular = np.expand_dims(x_regular, axis=-1)
        y_regular = np.expand_dims(y_regular, axis=-1)
        path_final = np.concatenate([x_regular, y_regular], axis = 1)
        return x_regular,y_regular
    except Exception as e:
        # print(len(t),len(x))
        return None,None
    # tck = interpolate.splrep(t, x, s = smallS, k = 5)
    # x_new = interpolate.splev(t, tck, der=0)
    # y = np.array(path_y)
    # tck = interpolate.splrep(t, y, s = smallS, k = 5)
    # y_new = interpolate.splev(t, tck, der=0)
    # distance = np.cumsum(np.sqrt( np.ediff1d(x_new, to_begin=0)**2 + np.ediff1d(y_new, to_begin=0)**2 ))
    # n_points=50
    # distance = distance/distance[-1]
    # fx, fy = interpolate.interp1d( distance, x_new ), interpolate.interp1d( distance, y_new )
    # alpha = np.linspace(0, 1, n_points)
    # x_regular, y_regular = fx(alpha), fy(alpha)
    # x_regular = np.expand_dims(x_regular, axis=-1)
    # y_regular = np.expand_dims(y_regular, axis=-1)
    # path_final = np.concatenate([x_regular, y_regular], axis = 1)
    # return(x_regular,y_regular)


def find_grad_sobel(img):
    X=cv.Sobel(img,cv.CV_64F,1,0,5)
    Y=cv.Sobel(img,cv.CV_64F,0,1,5)
    Z=np.ones_like(X)
    der=np.stack((X,Y,Z),axis=2)
    return der

def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def dot(x,y):
    return np.sum(x*y)

def listit(t):
        return list(map(listit, t)) if isinstance(t, (list, tuple)) else t

def tupleit(t):
        return tuple(map(tupleit, t)) if isinstance(t, (list, tuple)) else t

def bounding_ellipse(mask, center = None):
    mask = np.uint8(mask)
    contours = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    minEllipse = [None]*len(contours)
    for i, c in enumerate(contours):
        if c.shape[0] > 5:
            minEllipse[i] = cv.fitEllipse(c)

    drawing = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)

    if(len(minEllipse)):
        minEllipse = listit(minEllipse)
        minEllipse[0][1][0] *= 3
        minEllipse[0][1][1] *= 1.5
        minEllipse = tupleit(minEllipse)
        cv.ellipse(drawing, minEllipse[0], 255, -1)

    drawing = cv.cvtColor(drawing, cv.COLOR_BGR2GRAY)
    thresh = cv.threshold(drawing, 20, 255, cv.THRESH_BINARY)[1]
    surr = thresh - mask
    return surr

def find_path_with_car():
    global mask,dep,center, pre_est
    surr = bounding_ellipse(mask, center)
    norms=find_grad_sobel(dep)
    norms=normalized(norms,axis=2)
    norms=project_normals(norms)
    norm_mean = np.mean(norms[surr==255], axis=0)
    norm_mean /= np.linalg.norm(norm_mean)
    pre_est = norm_mean
    dp = norms*norm_mean
    dp = np.sum(dp, axis=2)
    lane = dp > 0.9
    lane[mask == 255] = 1
    lane=binary_erosion(lane,disk(10))
    lane = np.uint8(lane)
    (numLabels, labels, stats, centroids)=cv.connectedComponentsWithStats(lane,4)
    mx=0
    idx=-1
    for i in range(1,numLabels):
        if stats[i, cv.CC_STAT_AREA]>mx:
            mx = stats[i, cv.CC_STAT_AREA]
            idx=i
    lane=labels==idx
    global lane_mask
    lane_mask=255*np.uint8(lane)
    edges=skeletonize(lane^binary_erosion(lane,disk(5)))
    _,labels=cv.connectedComponents(np.uint8(edges),8)
    x,y=np.where(labels==1)
    lx,ly=fit_spline(x,y)
    if lx is None:
        return None
    x,y=np.where(labels==2)
    rx,ry=fit_spline(x,y)
    if rx is None:
        return None
    mx,my=(lx+rx)/2,(ly+ry)/2
    mx1,my1=fit_spline(mx,my)
    mx1=np.int32(mx1)
    my1=np.int32(my1)
    return [lx,ly,rx,ry,mx1,my1]

def find_path_without_car(dep):
    global lane_mask,pre_est
    norms=find_grad_sobel(dep)
    norms=normalized(norms,axis=2)
    norms=project_normals(norms)
    dp=np.sum(norms*pre_est,axis=2)
    dp[np.isnan(dp)] = 0
    lane=dp>0.995
    lane=binary_erosion(lane,disk(5))
    lane=np.uint8(lane)
    (numLabels, labels, stats, centroids)=cv.connectedComponentsWithStats(lane,4)
    mx=0
    idx=-1
    for i in range(1,numLabels):
        if stats[i, cv.CC_STAT_AREA]>mx:
            mx = stats[i, cv.CC_STAT_AREA]
            idx=i
    lane=labels==idx
    lane_mask=255*np.uint8(lane)
    if not np.any(lane):
        return None
    pre_est=np.mean(norms[lane], axis=0)
    edges=skeletonize(lane^binary_erosion(lane,disk(5)))
    _,labels=cv.connectedComponents(np.uint8(edges),8)
    x,y=np.where(labels==1)
    lx,ly=fit_spline(x,y)
    if lx is None:
        return None
    x,y=np.where(labels==2)
    rx,ry=fit_spline(x,y)
    if rx is None:
        return None
    mx,my=(lx+rx)/2,(ly+ry)/2
    mx1,my1=fit_spline(mx,my)
    mx1=np.int32(mx1)
    my1=np.int32(my1)
    return [lx,ly,rx,ry,mx1,my1]

def callback(data):
    bridge=CvBridge()
    try:
        frame=bridge.imgmsg_to_cv2(data,"32FC1")
    except CvBridgeError as e:
        print(e)
    global dep
    dep=np.array(frame,dtype=np.float32)

def carback(data):
    global iscar
    if data:
        iscar=False
    else:
        iscar=True

def maskback(data):
    bridge=CvBridge()
    try:
        frame=bridge.imgmsg_to_cv2(data,"8UC1")
    except CvBridgeError as e:
        print(e)
    global mask
    mask=np.array(frame,dtype=np.float32)

def project_normals(norms):
    new_norms=norms.reshape((-1,3))
    new_norms=new_norms.T
    global drone_pose
    quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
    rot = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    new_norms = np.matmul(rot, new_norms)
    new_norms=new_norms.T
    new_norms=new_norms.reshape((480,640,3))
    # new_norms=new_norms.reshape((2,3,3))
    return new_norms

def projection(cx,cy,img,K=None): #takes 2 arrays of dim 1xn of x and y pixel coordinates and returns local 3d coordinates
    # Inputs:
    # cx, cy: Points in Pixel coordinates
    cx=cx.reshape(-1)
    cy=cy.reshape(-1)
    # TODO: Identify why cx cy is out of range
    mask=cx<480
    mask=mask&(cx>=0)
    mask=mask&(cy<640)
    mask=mask&(cy>=0)
    cx=cx[mask]
    cy=cy[mask]

    cx=np.int32(cx)
    cy=np.int32(cy)

    ## Correction for swaping image frame coordinates and array indices
    X=np.vstack((cy,cx,np.ones(len(cx))))
    ####

    # print(X.shape)
    # im_center=np.array([cX,cY,1])
    # im_center=im_center.reshape((3,1))
    k_int = np.array([[554.254691191187, 0.0, 320.5],
                        [0.0, 554.254691191187, 240.5],
                        [0.0, 0.0, 1.0]])
    if K is not None:
        k_int=K
    X=np.matmul(np.linalg.inv(k_int),X)
    unit_vec=X/np.linalg.norm(X,axis=0)
    dep=img[cx,cy]
    # print("UNIT VEC",unit_vec)
    # print("DEPTH",dep)
    new_vec=unit_vec*dep
    global drone_pose
    # transform_mat=np.eye(4)
    quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
    mat = tf.transformations.quaternion_matrix(quaternion)
    transform_mat=mat
    transform_mat[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
    new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
    # new_vec_p[:3] = new_vec
    transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                    [-1., 0., 0., 0.],
                                    [0., 0., -1., 0.],
                                    [0., 0., 0., 1.]])
    # transform_mat_c_d=np.eye(4)
    # print("NEW", new_vec_p)
    coord = np.matmul(transform_mat_c_d, new_vec_p)
    # print("COORD IN DRNE FRAME", coord)
    # print("MAT",transform_mat)
    coord=np.matmul(transform_mat, coord)
    # print("COORD IN WORLD FRAME", coord.T)
    return coord

def poseback(data):
    global drone_pose
    drone_pose=data

def publish_traj(pub,cx,cy,name):
    global dep
    path=projection(cx,cy,dep)[:3]
    traj=JointTrajectory()
    traj.joint_names=name
    traj.header.stamp=rospy.Time.now()
    for i in range(path.shape[1]):
        pt=JointTrajectoryPoint()
        pt.positions=path[:,i]
        traj.points.append(pt)
    pub.publish(traj)

def publish_norm(pub):
    vect = Vector3()
    vect.x = pre_est[0]
    vect.y = pre_est[1]
    vect.z = pre_est[2]
    pub.publish(vect)


def segmenter():
    rospy.init_node('segmenter')
    rospy.Subscriber('/car_state/is_car',Bool,carback)
    rospy.Subscriber('/car_state/mask',Image,maskback)
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseback)
    left_pub=rospy.Publisher('/lane/left',JointTrajectory,queue_size=1)
    right_pub=rospy.Publisher('/lane/right',JointTrajectory,queue_size=1)
    mid_pub=rospy.Publisher('/lane/mid',JointTrajectory,queue_size=1)
    mask_pub=rospy.Publisher('/lane/mask',Image,queue_size=1)
    norm_pub=rospy.Publisher('/lane/norm',Vector3,queue_size=1)
    bridge=CvBridge()
    while not rospy.is_shutdown():
        start_time=time.time()
        global iscar, dep
        try:
            if iscar == True:
                pass
        except Exception as e:
            iscar=False
        if iscar:
            path=find_path_with_car(dep)
        else:
            path=find_path_without_car(dep)
        end_time=time.time()

        if path is not None:
            # print(path[4].shape)
            publish_traj(left_pub,path[0],path[1],"LEFT")
            publish_traj(right_pub,path[2],path[3],"RIGHT")
            publish_traj(mid_pub,path[4],path[5],"MID")
            mask_pub.publish(bridge.cv2_to_imgmsg(lane_mask))
            publish_norm(norm_pub)
            # pre_est1=np.array([1., 0., 0.]).reshape((1,1,3))
            # test=np.array([[pre_est, pre_est1, pre_est],
            #                 [pre_est1, pre_est, pre_est1]])
            # print(project_normals(test))

if __name__ == '__main__':
    segmenter()