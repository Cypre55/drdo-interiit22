#!/usr/bin/env python
import geometry_msgs.msg
import rospy
import std_msgs
import geometry_msgs
import sensor_msgs
import cv_bridge
import tf.transformations
import message_filters
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import tf
import math
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from tf.transformations import quaternion_matrix, quaternion_from_euler
from drdo_interiit22.msg import customMessage

# import sys
# sys.path.append("../")
# from projection2Dto3D import projection

pub1 = rospy.Publisher('car_state/complete', customMessage, queue_size=10)
# pub2 = rospy.Publisher('car_state/mask_contour', sensor_msgs.msg.Image, queue_size=10)
pub2 = rospy.Publisher('car_state/detected_contour', sensor_msgs.msg.Image, queue_size=10)

drone_pose = PoseStamped()

global t0
global last_x, last_y, last_z
xdata = []
ydata = []
zdata = []
time = []
xvel = []
yvel = []
zvel = []
dist = []


def rgbCallback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    global imgimg, header_imgimg
    header_imgimg = data.header
    imgimg = cv_image


def depthCallback(data):
    bridge = CvBridge()
    try:
        cv_depth = bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print(e)
    global im2, header_im2
    header_im2 = data.header
    im2 = cv_depth


def poseCallback(data):
    global drone_pose
    # headerInfo = data.header
    # rmat = quaternion_matrix([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
    # rmat[0][3] = data.pose.position.x
    # rmat[1][3] = data.pose.position.y
    # rmat[2][3] = data.pose.position.z
    drone_pose = data


def maskCallback(data):
    global cv_mask
    bridge = CvBridge()
    try:
        cv_mask = bridge.imgmsg_to_cv2(data, "8UC1")
    except CvBridgeError as e:
        print(e)
    global maskimg, header_maskimg
    header_maskimg = data.header
    maskimg = cv_mask


# kalman filter data
#
#
# def kalman():
#
#     for i in range(xdata.size):
#         z = np.array([[xdata[i], ydata[i], zdata[i]]], dtype=np.float32).T
#         y = z - H @ X
#         S = H @ (P @ H.T) + R
#         Sinv = np.linalg.inv(S)
#         k = P @ ((H.T) @ Sinv)
#         X = X + k @ y
#         P = (I - k @ H) @ P
#         X = F @ X + u
#         P = F @ (P @ F.T)
#         xpred.append(X[0, 0])
#         ypred.append(X[1, 0])
#         zpred.append(X[2, 0])
#         # x y z vx vy vz ax ay az

def detector():
    rospy.init_node('detector')
    global imgimg, im2, drone_pose, cv_mask
    imgimg = None
    im2 = None
    drone_pose = None
    cv_mask = None
    # strTime = rospy.get_time()
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, depthCallback)
    rospy.Subscriber("/depth_camera/rgb/image_raw", Image, rgbCallback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/lane/mask", Image, maskCallback)
    # print("Subscribed")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if imgimg is not None and im2 is not None and drone_pose is not None and cv_mask is not None:
            calculations()
            imgimg = None
            im2 = None
            drone_pose = None
            cv_mask = None
        rate.sleep()


def gradients(cy,cx):
    global im2
    # print("Gradients called")
    # points is a 3x1 matrix of (cy,cx,1)
    points = np.array([cy,cx,1])
    points = np.reshape(points,(3,1))
    cv_depth = im2

    # pixels in image x,y
    # x = 124
    # y = 134

    # coord = projection(np.array([cy, cy, cy+1]), np.array([cx , cx+1 ,cx]), cv_depth, drone_pose)
    Z = cv_depth[cx,cy]
    Zx = cv_depth[cx+1,cy] - cv_depth[cx,cy]
    Zy = cv_depth[cx,cy+1] - cv_depth[cx,cy]
    k_int = np.array([[554.254691191187, 0.0, 320.5],
                      [0.0, 554.254691191187, 240.5],
                      [0.0, 0.0, 1.0]])
    # Z = coord[0,2]
    # Zx = coord[1,2] - Z
    # Zy = coord[2,2] - Z
    f = k_int[0][0]
    c_x = k_int[0][2]
    c_y = k_int[1][2]
    xx = Z/f + (cx-c_x)*Zx/f
    xy = (cx-c_x)*Zy/f
    yx = (cy-c_y)*Zx/f
    yy = Z/f + (cy-c_y)*Zy/f
    #
    vx = np.array([xx,yx,Zx,1.0])
    vy = np.array([xy,yy,Zy,1.0])
    vx = np.reshape(vx,(4,1))
    vy = np.reshape(vy,(4,1))
    # print("VX")
    # print(vx)
    # print("VY")
    # print(vy)
    # # print("VX")
    # # print(vx.shape)
    # # print("VY")
    # # print(vy)
    # normal1 = np.cross(vx,vy)
    # print("Normal 1")
    # print(normal1)
    #




    # k_int_inv = np.array([[0.00180422, 0.0, -0.57825401],
    #                       [0.0, 0.00180422, -0.43391604],
    #                       [0.0, 0.0, 1.0]])
    transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                  [-1., 0., 0., 0.],
                                  [0., 0., -1., 0.],
                                  [0., 0., 0., 1.]])
    # global drone_pose
    # transform_mat=np.eye(4)
    quaternion = (drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z,
                  drone_pose.pose.orientation.w)
    mat = tf.transformations.quaternion_matrix(quaternion)
    transform_mat = mat
    transform_mat[:3, 3] = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]

    transform_new = np.matmul(transform_mat_c_d,transform_mat)
    vx_new = np.matmul(transform_new,vx)
    vy_new = np.matmul(transform_new,vy)
    # print("new v")
    vx_new = vx_new[:3]
    vy_new = vy_new[:3]
    vx_new = np.reshape(vx_new,(1,3))
    vy_new = np.reshape(vy_new,(1,3))
    # print(vx_new)
    # print(vy_new)
    normal = np.cross(vx_new,vy_new)
    # normal = np.reshape(normal,(1,4))

    # print(transform_mat_c_d.shape)

    # print(k_int_inv.shape)
    # k_int_inv = np.vstack((k_int_inv,np.ones(3)))
    # print(k_int_inv.shape)
    # alpha = np.matmul(transform_mat_c_d,k_int_inv)
    # alpha = np.matmul(transform_mat,alpha)
    #
    # gamma = np.matmul(k_int_inv,points)
    # gamma = np.linalg.norm(gamma,axis=0)
    # print("alpha")
    # print(alpha)
    # a = np.power(k_int_inv[1][1],2)
    # b = np.power(k_int_inv[0][0],2)
    # d = k_int_inv[1][1]
    # e = 2*k_int_inv[1][1]*k_int_inv[1][2]
    # f = 2*k_int_inv[0][0]*k_int_inv[0][2]
    # c = 0
    # Xx = (alpha*gamma - (alpha*cx*(2*a*cx + e))/(2*gamma))/(np.power(gamma,2))
    # Yx = (-2*a*alpha*cx*cy)/(np.power(gamma,2))
    # Xy = (-2*b*alpha*cx*cy)/(np.power(gamma,2))
    # Yy = (alpha*gamma - (alpha*cy*(2*b*cy + f))/(2*gamma))/(np.power(gamma,2))
    #
    # Vx = np.array([Xx,Yx,Zx])
    # print("Vx")
    # print(Vx)
    # Vy = np.array([Xy,Yy,Zy])
    # Vx = np.reshape(Vx,(1,3))
    # Vy = np.reshape(Vy,(1,3))
    # # print("VX")
    # # print(Vx.shape)
    # # print(Vx)
    # # print("VY")
    # # print(Vy.shape)
    # # print(Vy)
    # normal = np.cross(Vx,Vy)

    # print("normal")
    # print(normal)
    return normal

def projection(cx, cy, img,
               K=None):  # takes 2 arrays of dim 1xn of x and y pixel coordinates and returns local 3d coordinates
    # Inputs:
    # global quaternion,transform_mat,transform_mat_c_d
    # cx, cy: Points in Pixel coordinates

    cx = cx.reshape(-1)
    cy = cy.reshape(-1)
    # TODO: Identify why cx cy is out of range
    mask = cx < 480
    mask = mask & (cx >= 0)
    mask = mask & (cy < 640)
    mask = mask & (cy >= 0)
    cx = cx[mask]
    cy = cy[mask]

    cx = np.int32(cx)
    cy = np.int32(cy)

    ## Correction for swaping image frame coordinates and array indices
    X = np.vstack((cy, cx, np.ones(len(cx))))
    ####

    # print(X.shape)
    # im_center=np.array([cX,cY,1])
    # im_center=im_center.reshape((3,1))
    k_int = np.array([[554.254691191187, 0.0, 320.5],
                      [0.0, 554.254691191187, 240.5],
                      [0.0, 0.0, 1.0]])
    # if K is not None:
    #     k_int=K
    # print("X1", X)
    # print("kinv")
    # print(np.linalg.inv(k_int))
    X = np.matmul(np.linalg.inv(k_int), X)
    # print("X2", X)
    # print("X")
    # print(X)
    unit_vec = X / np.linalg.norm(X, axis=0)
    # print("norm")
    # print(np.linalg.norm(X, axis=0))
    # print("Unit vector")
    # print(unit_vec)
    dep = img[cx, cy]

    # print("UNIT VEC",unit_vec)
    # print("DEPTH",dep)
    # print("HERE")
    # print(unit_vec)

    # print(dep)
    new_vec = unit_vec * dep
    # print(new_vec)
    global drone_pose
    # transform_mat=np.eye(4)
    quaternion = (drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z,
                  drone_pose.pose.orientation.w)
    mat = tf.transformations.quaternion_matrix(quaternion)
    transform_mat = mat
    transform_mat[:3, 3] = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
    new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
    # print("new_vec_p")
    # print(new_vec_p)
    # new_vec_p[:3] = new_vec
    transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                  [-1., 0., 0., 0.],
                                  [0., 0., -1., 0.],
                                  [0., 0., 0., 1.]])
    # transform_mat_c_d=np.eye(4)
    # print("NEW", new_vec_p)
    coord = np.matmul(transform_mat_c_d, new_vec_p)
    # print("coord")
    # print(coord)
    # print("MAT",transform_mat)
    # print("COORD IN DRNE FRAME", coord)
    coord = np.matmul(transform_mat, coord)
    # print("COORD IN WORLD FRAME", coord.T)
    # print("transofr")
    # print(transform_mat)
    # print("ccoord")
    # print(coord)
    return coord
    # err = coord.T[0][:2] - prius_pose[i][:, 3][:2]
    # errs_list.append(np.linalg.norm(err))






"""
def gradients(image,f,cx,cy):
    f = 240
    cx = 0
    cy = 0

    x = 34
    y = 52

    p = (x, y)
    px = (x + 1, y)
    py = (x, y + 1)

    # Z = depth(x, y, image)
    # Zx = depth(x + 1, y, image) - Z
    # Zy = depth(x, y + 1, image) - Z
    Z = image[p[0],p[1]]

    Zx = image[p[0]+1,p[1]] - Z
    Zy = image[p[0],p[1]+1] - Z
    # coord = projection(np.array[p[1],px[1],py[1]],np.array[p[1],px[1],py[1]],image)

    # pinhole camera

    X = (x - cx) * Z / f
    Y = (y - cy) * Z / f


    Xx = Z / f + X * Zx / Z
    Yx = Y * Zx / Z

    Xy = X * Zy / Z
    Yy = Z / f + Y * Zy / Z

    vx = np.array([Xx, Yx, Zx])
    vy = np.array([Xy, Yy, Zy])

    normal = np.cross(vx, vy)

    return normal

"""


# 2 rgb
# 3 local pose
def calculations():
    # print("CALLBACK CALLED")
    startTime = rospy.get_time()
    bridge = CvBridge()
    global imgimg, im2, drone_pose, header_im2, header_imgimg
    # cv_temp = np.array(im2,dtype=np.float)
    # cv2.normalize(cv_temp,cv_temp,0,255,cv2.NORM_MINMAX)
    # cv2.imshow("depth",cv_temp)
    # cv2.imwrite("depth.png",cv_temp)
    # cv2.waitKey(0)
    # cv2.imshow("rgb",imgimg)
    # cv2.imwrite("rgb.png",imgimg)
    # cv2.waitKey(0)
    # try:
    #     cv_image = bridge.imgmsg_to_cv2(data2, "bgr8")
    #     cv_depth = bridge.imgmsg_to_cv2(data1, "32FC1")
    # except CvBridgeError as e:
    #     print(e)
    # drone_pose = data3
    # rmat = quaternion_matrix([data3.pose.orientation.w,data3.pose.orientation.x,data3.pose.orientation.y,data3.pose.orientation.z])
    # R = rmat[:3,:3]
    # print(rmat)
    image = imgimg
    cv_depth = im2

    # cv2.imshow("testerimage",image)
    # cv2.waitKey(0)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # blur = cv2.medianBlur(image, 7)
    # gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    # gray = cv2.bitwise_not(gray)

    # thresh = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY_INV)[1]
    # if cv_mask != None:
    # Taking a matrix of size 5 as the kernel
    
    kernel = np.ones((3,3), np.uint8)
    
    cv_mask_dil = cv2.dilate(cv_mask, kernel, iterations=1)
    
    thresh = 255 - cv_mask_dil
    # cv2.imshow("mask_inv",thresh)
    # cv2.imwrite("mask_inv.png",thresh)
    # cv2.waitKey(0)
    # else:
    #     return

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # CHANGE AND TRY
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 400
    max_area = 40000

    pubMsg = customMessage()
    pubMsg.header = header_imgimg
    pubMsg.isMaskDetected.data = False
    pubMsg.isCarNinety.data = False
    # isMask = False
    # gl_arr=[]

    #############################
    for cnt in cnts:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        M = cv2.moments(cnt)
        area = cv2.contourArea(cnt)
        cx=cy=0
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

        if rect[1][0] == 0 or rect[1][1] == 0:
            continue
        area_ratio = area / (rect[1][0] * rect[1][1])

        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
        # approx = cv2.approxPolyDP(cnt, 0.005 * cv2.arcLength(cnt, True), True)
        # ellipse = cv2.fitEllipse(approx)
        ratioE = 0
        # print(len(approx))
        # print(approx.shape)
        if (approx.shape[0] >= 5):
            (x, y), (MA, ma), angle = cv2.fitEllipse(approx)

            ellipse = cv2.fitEllipse(approx)
            # area = cv2.contourArea(cnt)
            x = int(x)
            y = int(y)
            if MA==0:
                condition = False
            
            ratioE = ma/MA
            condition = ratioE>1.5 and ratioE<2.4
            print(ratioE)
        else:
            condition = True

        # print(area)
        if  area_ratio>0.8 and area > min_area and area < max_area and condition: #area_ratio > 0.8 and
            pubMsg.isMaskDetected.data = True
            # cv2.drawContours(image, [box],0,(200,0,0),3,)
            cv2.drawContours(image, [approx], 0, (0, 200, 0), 2 )
            # cv2.drawContours(image, [ellipse], 0, (200, 0, 0), 2 )
            
            if ratioE != 0:
                image = cv2.ellipse(image,ellipse,(0,255,0),2)
                print(ratioE)
                
                cv2.putText(image, '{}'.format(ratioE), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            # cv2.imshow("IMAGE",image)
            # cv2.waitKey(0)
            # cv2.imwrite("car_contour_bounding_rect.png",image)
            cv2.circle(image, (int(cx), int(cy)), 7, (0, 0, 255), -1)
            # cv2.imwrite("car_contour_bounding_rect_centre.png",image)
            # CREATING MASK OF CONTOURx
            img = image[:, :, 0].astype('uint8')
            mask = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask, [cnt], -1, (255), thickness=-1)
            # cv2.imshow("image",image)
            # cv2.waitKey(0)


            # imgs = image[:, :, 0].astype('uint8')
            mask_rect = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask_rect, [box], 0, (255), thickness=-1)
            subtracted = cv2.subtract(mask_rect, cv_mask)
            ret2, th2 = cv2.threshold(subtracted, 100, 255, cv2.THRESH_BINARY)
            # subtracted = cv2.subtract(mask_rect,th2)
            # cv2.imwrite("subtracted_mask.png",subtracted)
            white = np.argwhere(th2 == 255)
            white = np.transpose(white)
            # cv2.imshow("thresh",th2)
            # cv2.imshow("subtracted",subtracted)
            # cv2.imshow("cv_mask",cv_mask)
            # cv2.waitKey(0)
            # print("Hi")
            # cv2.imshow("subtracted",subtracted)
            y = np.mean(white, axis=1)
            # cv2.circle(image, (int(y[1]), int(y[0])), 7, (0, 255, 255), -1)
            dist1 = (box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2
            dist2 = (box[1][0] - box[2][0]) ** 2 + (box[1][1] - box[2][1]) ** 2
            # xfront = yfront = xback = yback =0
            if dist1 > dist2:
                xtop = (box[1][0] + box[2][0]) / 2
                xbot = (box[0][0] + box[3][0]) / 2
                ybot = (box[0][1] + box[3][1]) / 2
                ytop = (box[1][1] + box[2][1]) / 2
            else:
                xtop = (box[1][0] + box[0][0]) / 2
                xbot = (box[2][0] + box[3][0]) / 2
                ybot = (box[2][1] + box[3][1]) / 2
                ytop = (box[1][1] + box[0][1]) / 2
    #############################
    # for cnt in cnts:
    #     rect = cv2.minAreaRect(cnt)
    #     box = cv2.boxPoints(rect)
    #     box = np.int0(box)

    #     M = cv2.moments(cnt)
    #     area = cv2.contourArea(cnt)
    #     cx = cy = 0
    #     if M['m00'] != 0:
    #         cx = int(M['m10'] / M['m00'])
    #         cy = int(M['m01'] / M['m00'])

    #     if rect[1][0] == 0 or rect[1][1] == 0:
    #         continue
    #     area_ratio = area / (rect[1][0] * rect[1][1])

    #     approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
    #     # area = cv2.contourArea(cnt)

    #     # print(area)
    #     if area_ratio > 0.8 and area > min_area and area < max_area:  # area_ratio > 0.8 and
    #         pubMsg.isMaskDetected.data = True
    #         cv2.drawContours(image, [box], 0, (255, 0, 0), 3, )

    #         # cv2.drawContours(image, [approx], 0, (0, 200, 0), 2, )
    #         # cv2.waitKey(0)
    #         # cv2.circle(image, (int(cx), int(cy)), 7, (0, 0, 255), -1)
    #         # CREATING MASK OF CONTOURx
    #         img = image[:, :, 0].astype('uint8')
    #         mask = np.zeros(img.shape, np.uint8)
    #         cv2.drawContours(mask, [cnt], -1, (255), thickness=-1)
    #         # cv2.imshow("image",image)
    #         # cv2.waitKey(0)

    #         # imgs = image[:, :, 0].astype('uint8')
    #         mask_rect = np.zeros(img.shape, np.uint8)
    #         cv2.drawContours(mask_rect, [box], 0, (255), thickness=-1)
    #         subtracted = cv2.subtract(mask_rect, cv_mask)
    #         ret2, th2 = cv2.threshold(subtracted, 100, 255, cv2.THRESH_BINARY)
    #         # cv2.imshow("thresh2",th2)
    #         # cv2.waitKey(0)
    #         # cv2.imwrite("car_contour_gray.png",th2)
    #         # img_bgr = cv2.cvtColor(th2, cv2.COLOR_GRAY2BGR)
    #         # cv2.drawContours(img_bgr,[box],0,(255,0,0),thickness =3,)
    #         subtracted = cv2.subtract(mask_rect, th2)
    #         # cv2.imshow("mask_rect",mask_rect)
    #         # cv2.waitKey(0)
    #         white = np.argwhere(th2 == 255)
    #         white = np.transpose(white)
    #         # cv2.imshow("thresh2_rect",img_bgr)
    #         # cv2.imwrite("car_contour_rect.png",img_bgr)
    #         # cv2.imshow("subtracted.png",subtracted)
    #         # cv2.imwrite("subtracted.png",subtracted)
    #         # cv2.imshow("cv_mask.png",cv_mask)
    #         # cv2.imwrite("input.png",cv_mask)
    #         # cv2.waitKey(0)
    #         # print("Hi")
    #         # cv2.imshow("subtracted",subtracted)
    #         y = np.mean(white, axis=1)
    #         # cv2.circle(image, (int(y[1]), int(y[0])), 7, (0, 255, 255), -1)
    #         dist1 = (box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2
    #         dist2 = (box[1][0] - box[2][0]) ** 2 + (box[1][1] - box[2][1]) ** 2
    #         # xfront = yfront = xback = yback =0
    #         if dist1 > dist2:
    #             xtop = (box[1][0] + box[2][0]) / 2
    #             xbot = (box[0][0] + box[3][0]) / 2
    #             ybot = (box[0][1] + box[3][1]) / 2
    #             ytop = (box[1][1] + box[2][1]) / 2
    #         else:
    #             xtop = (box[1][0] + box[0][0]) / 2
    #             xbot = (box[2][0] + box[3][0]) / 2
    #             ybot = (box[2][1] + box[3][1]) / 2
    #             ytop = (box[1][1] + box[0][1]) / 2

            dist1 = (xtop - y[1]) ** 2 + (ytop - y[0]) ** 2
            dist2 = (xbot - y[1]) ** 2 + (ybot - y[0]) ** 2
            if dist1 > dist2:

                # cv2.line(image, (int(cx), int(cy)), (int(xtop), int(ytop)), (0, 0, 255), 3)
                angle = np.arctan2(ybot - cy, xbot - cx)
                # print(ybot - cy)
                # print(xbot - cx)
                xback = xbot
                yback = ybot
                xfront = xtop
                yfront = ytop
                # SWAPPED X Y for Projection to work
                # coord = projection(np.array([ybot, cy, ytop]), np.array([xbot, cx, xtop]), cv_depth, drone_pose)
                # coord = projection(np.array([xtop]), np.array([ytop]), cv_depth, drone_pose)
                # cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
                # cv2.circle(image,(int(xbot),int(ybot)),7,(255,0,0),-1)
                # cv2.circle(image,(int(xtop),int(ytop)),7,(0,255,255),-1)
            else:
                # cv2.line(image, (int(xbot), int(ybot)), (int(cx), int(cy)), (0, 0, 255), 3)

                angle = np.arctan2(ytop - cy, xtop - cx)
                # print(ybot - cy)
                # print(xbot - cx)
                xback = xtop
                yback = ytop
                xfront = xbot
                yfront = ybot
                # SWAPPED X Y for Projection to work
                # coord = projection(np.array([ytop, cy, ybot]), np.array([xtop, cx, xbot]), cv_depth, drone_pose)
                # coord = projection(np.array([xbot]), np.array([ybot]), cv_depth, drone_pose)
                # cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
                # cv2.circle(image,(int(xtop),int(ytop)),7,(255,0,0),-1)
                # cv2.circle(image,(int(xbot),int(ybot)),7,(0,255,255),-1)
            if cx>32 and cx<608 and cy>24 and cy<456:
                pubMsg.isCarNinety.data = True


            coord = projection(np.array([yfront,cy,yback]), np.array([xfront,cx,xback]), cv_depth, drone_pose)
            # cv2.circle(image,(int(cx),int(cy)),7,(0,0,255),-1)
            # cv2.circle(image, (int(xfront), int(yfront)), 7, (0, 0,255), -1)
            # cv2.arrowedLine(image,(int(cx),int(cy)),(int(xfront), int(yfront)),(0,255,0),thickness = 4)
            # normal = gradients(cy+35,cx+35)
            # normal2 = gradients(cy + 70,cx+70)
            # print("Should be 0")
            # print(np.cross(normal,normal2))

            # normal = gradients(cx+35,cy+35)
            # normal2 = gradients(cx + 200,cy+100)
            # print("Should be actually 0")
            # print(np.cross(normal,normal2))
            # cv2.circle(image,(int(cx+35),int(cy+35)),7,(255,255,255),-1)
            # cv2.circle(image,(int(cx-250),int(cy)),7,(255,255,255),-1)
            # cv2.imshow("image",image)
            # cv2.waitKey(0)
            # cv2.circle(image,(int(xfront),int(yfront)),7,(255,0,0),-1)
            # cv2.imshow("image",image)
            # cv2.imwrite("final.png",image)
            # cv2.waitKey(0)
            # angle = angle * 180 / np.pi
            # x_y_yaw = str(cx) + "," + str(cy) + "," + str(angle)
            # cv2.putText(image, x_y_yaw, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # print("x, y, yaw")
            # print(cx, cy, angle)
            # cv2.imshow("image",image)
            # cv2.waitKey(1)

            # result = [cx, cy, angle]
            # cv2.imshow("images", image)
            # cv2.waitKey(1)
            pub2.publish(bridge.cv2_to_imgmsg(image))

            # ARCHIT CODE
            # print("Number of Points")
            # print(cx,cy,xfront,yfront,xback,yback)
            # cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
            # cv2.circle(image,(int(xfront),int(yfront)),7,(255,0,127),-1)
            # cv2.circle(image,(int(xback),int(yback)),7,(255,0,127),-1)
            # cv2.imshow("points",image)

            # isMask = True
            if coord.shape[1] == 3:
                # isMask = True
                dY = coord[1, 0] - coord[1, 2]
                dX = coord[0, 0] - coord[0, 2]

                yaw_angle = np.arctan2(dY, dX)
                # final_tf = quaternion_matrix([np.sin(angle/2.0), np.cos(angle/2.0) ,0, 0,])
                # final_tf[0][3] = coord[0]
                # final_tf[1][3] = coord[1]
                # final_tf[2][3] = coord[2]
                # final_pose = geometry_msgs.msg.TransformStamped()
                # final_tf.header = data3.header
                # final_tf.transform.translation.x = coord[0]
                # final_tf.transform.translation.y = coord[1]
                # final_tf.transform.translation.z = coord[2]
                #
                # final_tf.transform.rotation.x = np.cos(angle/2.0)
                # final_tf.transform.rotation.y = 0
                # final_tf.transform.rotation.z = 0
                # final_tf.transform.rotation.w = np.sin(angle/2.0)
                # print("sec", header_im2.stamp.to_sec())
                # print("nsec", header_im2.stamp.to_nsec())

                vel_x = vel_y = vel_z = 0
                if len(xdata) != 0:
                    # print(xdata)
                    dT = header_im2.stamp.to_sec() - time[-1]
                    dT = max(dT, 1e-3)
                    # print("DT: ", dT)
                    vel_x = (coord[0, 2] - xdata[-1]) / (dT)
                    vel_y = (coord[1, 2] - ydata[-1]) / (dT)
                    vel_z = (coord[2, 2] - zdata[-1]) / (dT)
                    # print("VEL at correct case", vel_x, vel_y)
                else:
                    vel_x = 0
                    vel_y = 0
                    vel_z = 0
                    # print("VEL at 0 case", vel_x, vel_y)

                # dist_temp = (xdata[-1]-coord_back[0])**2 + (ydata[-1]-coord_back[1])**2 + (zdata[-1]-coord_back[2])

                xdata.append(coord[0, 2])
                ydata.append(coord[1, 2])
                zdata.append(coord[2, 2])
                time.append(header_im2.stamp.to_sec())
                xvel.append(vel_x)
                yvel.append(vel_y)
                zvel.append(vel_z)



                pubMsg.car_state.header = header_im2
                pubMsg.car_state.pose.pose.position.x = coord[0, 2]
                pubMsg.car_state.pose.pose.position.y = coord[1, 2]
                pubMsg.car_state.pose.pose.position.z = coord[2, 2]

                pubMsg.car_centre.x = coord[0,1]
                pubMsg.car_centre.y = coord[1,1]
                pubMsg.car_centre.z = coord[2,1]

                pubMsg.car_back.x = coord[0,2]
                pubMsg.car_back.y = coord[1,2]
                pubMsg.car_back.z = coord[2,2]

                pubMsg.car_front.x = coord[0,0]
                pubMsg.car_front.y = coord[1,0]
                pubMsg.car_front.z = coord[2,0]

                # final_pose = geometry_msgs.msg.PoseStamped()
                # final_pose.pose.position.x = coord[0,2]
                # final_pose.pose.position.y = coord[1,2]
                # final_pose.pose.position.z = coord[2,2]
                # final_pose.header = drone_pose.header

                pubMsg.car_centre.x = coord[0,1]
                pubMsg.car_centre.y = coord[1,1]
                pubMsg.car_centre.z = coord[2,1]

                pubMsg.car_back.x = coord[0,2]
                pubMsg.car_back.y = coord[1,2]
                pubMsg.car_back.z = coord[2,2]

                pubMsg.car_front.x = coord[0,0]
                pubMsg.car_front.y = coord[1,0]
                pubMsg.car_front.z = coord[2,0]

                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)

                pubMsg.car_state.pose.pose.orientation.x = quaternion[0]
                pubMsg.car_state.pose.pose.orientation.y = quaternion[1]
                pubMsg.car_state.pose.pose.orientation.z = quaternion[2]
                pubMsg.car_state.pose.pose.orientation.w = quaternion[3]

                # final_vel = geometry_msgs.msg.TwistStamped()
                # final_vel.twist.linear.x = vel_x
                # final_vel.twist.linear.y = vel_y
                # final_vel.twist.linear.z = vel_z
                # final_vel.header = drone_pose.header

                pubMsg.car_state.twist.twist.linear.x = vel_x
                pubMsg.car_state.twist.twist.linear.y = vel_y
                pubMsg.car_state.twist.twist.linear.z = vel_z

                pubMsg.car_state.twist.twist.angular.x = 0
                pubMsg.car_state.twist.twist.angular.y = 0
                pubMsg.car_state.twist.twist.angular.z = 0



                pub1.publish(pubMsg)
                # pub2.publish(bridge.cv2_to_imgmsg(mask))

                # pub4.publish(final_vel)
                # pub3.publish(isMask)
                # pub1.publish(final_pose)
                # pub2.publish(bridge.cv2_to_imgmsg(mask))

            # else:
            #     isMask = False

            # cv2.imshow("mask",mask)
            # cv2.waitKey(0)

        else:
            # isMask = False
            pub1.publish(pubMsg)
        # reprojection
        # reproj=new_vec
        # reproj = np.dot(P, real_im_center)
        # print("actual is ",im_center,"reprojected is",reproj)
        # print("\nunit vec is \n",new_vec)
        # coord = np.array(coord)
        # gl_arr.append(coord)
        # print(coord)
        # print(prius_pose[i].T[3])
        # error = coord.T - prius_pose[i].T[3]
        # print(error[0])
        # print((error[0][0] + error[0][1]) / 2)


if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass
