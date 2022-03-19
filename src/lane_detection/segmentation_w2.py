#!/usr/bin/python
from cv2 import transform
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool,Float64MultiArray
from geometry_msgs.msg import Point,PoseStamped
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from skimage.morphology import binary_erosion,disk,skeletonize,binary_dilation,binary_closing
from scipy import ndimage as ndi
from scipy import interpolate
import time
import tf
from tf.transformations import quaternion_matrix
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import matplotlib.pyplot as plt
# from src.projection2Dto3D import projection

pre_est=np.array([0.,0.,1.]).reshape((1,1,3))
dep=np.zeros((480,640))
rgb=np.zeros((480,640,3))
mask=np.zeros((480,640))
lane_mask=np.zeros((480,640),dtype=np.uint8)
final_mask=np.zeros((480,640),dtype=np.uint8)
drone_pose=PoseStamped()
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
        tck = interpolate.splrep(t, x, s = smallS, k = 5)
        x_new = interpolate.splev(t, tck, der=0)
        y = np.array(path_y)
        tck = interpolate.splrep(t, y, s = smallS, k = 5)
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
        return None,None

def apply_kmeans(img, K=2):
    img = cv.bilateralFilter(img.astype(np.uint8),15,75,300)
    Z = img.reshape((-1,3))
    Z = np.float32(Z)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret,label,center=cv.kmeans(Z,K,None,criteria,10,cv.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))
    res2=cv.cvtColor(res2,cv.COLOR_BGR2GRAY)
    return res2

    # return label.reshape((480,640))

def biggest_connected_component(frame):
    nb_components, output, stats, centroids = cv.connectedComponentsWithStats(frame, connectivity=4)
    try:
        max_label, max_size = max([(i, stats[i, cv.CC_STAT_AREA]) for i in range(1, nb_components)], key=lambda x: x[1])
    except Exception as e:
        return None

    img2 = np.zeros(output.shape)
    img2[output == max_label] = 1
    return img2

def fill_holes(gray):
    if np.max(gray)==255:
        gray/=255
    des = cv.bitwise_not(gray)
    contour,hier = cv.findContours(np.uint8(des),cv.RETR_CCOMP,cv.CHAIN_APPROX_SIMPLE)[-2:]
    for cnt in contour:
        des=cv.drawContours(des,[cnt],0,1,-1)
    gray = cv.bitwise_not(des)
    return gray

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



def find_path_without_car(dep,rgb):
    global lane_mask,final_mask
    norms=find_grad_sobel(dep)
    norms=normalized(norms,axis=2)
    norms=project_normals(norms)
    dp=np.sum(norms*pre_est,axis=2)
    lane=dp>0.95
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
    kmean_output = apply_kmeans(rgb)
    thresh=np.int((np.int(np.max(kmean_output))+np.int(np.min(kmean_output)))/2)
    kmean_output[kmean_output > thresh] = 255  # a bit wrong, need to fix it
    kmean_output[kmean_output <= thresh] = 0
    kmean_output[np.isnan(dep)] = 0
    biggest_comp = biggest_connected_component(kmean_output)
    if biggest_comp is None:
        return None
    lane=np.array(biggest_comp,dtype=np.uint8)
    lane=np.uint8(fill_holes(biggest_comp))
    lane=binary_closing(lane,disk(5))
    final_mask=255*lane
    # cv.imshow('Lane',255*lane)
    # cv.waitKey(1)
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
    mask=mx1<480
    mask=mask&(my1<640)
    mx1=mx1[mask]
    my1=my1[mask]
    return [lx,ly,rx,ry,mx1,my1]

def callback(data):
    bridge=CvBridge()
    try:
        frame=bridge.imgmsg_to_cv2(data,"32FC1")
    except CvBridgeError as e:
        print(e)
    global dep
    dep=np.array(frame,dtype=np.float32)

def callback1(data):
    bridge=CvBridge()
    try:
        frame=bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)
    global rgb
    rgb=frame

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
    k_int = np.array([[554.254691191187, 0.0, 320.5],
                        [0.0, 554.254691191187, 240.5],
                        [0.0, 0.0, 1.0]])
    if K is not None:
        k_int=K
    X=np.matmul(np.linalg.inv(k_int),X)
    unit_vec=X/np.linalg.norm(X,axis=0)
    dep=img[cx,cy]
    new_vec=unit_vec*dep
    global drone_pose
    quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
    mat = tf.transformations.quaternion_matrix(quaternion)
    transform_mat=mat
    transform_mat[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
    new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
    transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                    [-1., 0., 0., 0.],
                                    [0., 0., -1., 0.],
                                    [0., 0., 0., 1.]])
    coord = np.matmul(transform_mat_c_d, new_vec_p)
    coord=np.matmul(transform_mat, coord)
    return coord

def poseback(data):
    global drone_pose
    drone_pose=data

def publish_traj(pub,cx,cy,name):
    if name=="LEFT":
        plt.clf()
    # plt.plot(cy,cx)
    # plt.draw()
    # plt.pause(0.00000000001)
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

def segmenter():
    global pub
    rospy.init_node('segmenter')
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, callback)
    rospy.Subscriber("/depth_camera/rgb/image_raw", Image, callback1)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseback)
    left_pub=rospy.Publisher('/lane/left',JointTrajectory,queue_size=1)
    right_pub=rospy.Publisher('/lane/right',JointTrajectory,queue_size=1)
    mid_pub=rospy.Publisher('/lane/mid',JointTrajectory,queue_size=1)
    mask_pub=rospy.Publisher('/lane/mask',Image,queue_size=1)
    pub=rospy.Publisher('/finalmask',Image,queue_size=1)
    bridge=CvBridge()
    while not rospy.is_shutdown():
        # start_time=time.time()
        global dep
        path=find_path_without_car(dep,rgb)
        mask_pub.publish(bridge.cv2_to_imgmsg(lane_mask))
        # end_time=time.time()
        if path is not None:
            publish_traj(left_pub,path[0],path[1],"LEFT")
            publish_traj(right_pub,path[2],path[3],"RIGHT")
            publish_traj(mid_pub,path[4],path[5],"MID")
            pub.publish(bridge.cv2_to_imgmsg(np.uint8(final_mask)))
        plt.ion()
        plt.show()

if __name__ == '__main__':
    segmenter()