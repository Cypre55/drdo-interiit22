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
from geometry_msgs.msg import PoseStamped,TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import tf
import math
import os
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer
from tf.transformations import quaternion_matrix
from drdo_interiit22.msg import customMessage

# import sys
# sys.path.append("../")
# from projection2Dto3D import projection


pub1 = rospy.Publisher('car_state/complete', customMessage, queue_size=10)
pub2 = rospy.Publisher('car_state/mask_contour', sensor_msgs.msg.Image, queue_size=10)

drone_pose = PoseStamped()

global t0
global last_x,last_y,last_z
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
    global imgimg, im2, drone_pose
    imgimg = None
    im2 = None
    drone_pose = None
    # strTime = rospy.get_time()
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, depthCallback)
    rospy.Subscriber("/depth_camera/rgb/image_raw", Image, rgbCallback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    # print("Subscribed")
    while not rospy.is_shutdown():
        if imgimg is not None and im2 is not None and drone_pose is not None:
            calculations()


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
    # print("X1", X)
    X=np.matmul(np.linalg.inv(k_int),X)
    # print("X2", X)
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
    # err = coord.T[0][:2] - prius_pose[i][:, 3][:2]
    # errs_list.append(np.linalg.norm(err))


# 2 rgb
# 3 local pose
def calculations():
    print("CALLBACK CALLED")
    startTime = rospy.get_time()
    bridge = CvBridge()
    global imgimg, im2, drone_pose, header_im2, header_imgimg
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

    blur = cv2.medianBlur(image, 7)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_not(gray)

    thresh = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY_INV)[1]

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # CHANGE AND TRY
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 0
    max_area = 9999990000
    
    pubMsg = customMessage()
    pubMsg.header = header_imgimg
    pubMsg.isMaskDetected.data = False
    # isMask = False
    # gl_arr=[]
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
        # area = cv2.contourArea(cnt)

        # print(area)
        if  area > min_area and area < max_area : #area_ratio > 0.8 and
            pubMsg.isMaskDetected.data = True
            cv2.drawContours(image, [approx], 0, (200, 0, 0), 3, )
            # cv2.circle(image, (int(cx), int(cy)), 7, (255, 255, 255), -1)
            # CREATING MASK OF CONTOURx
            img = image[:, :, 0].astype('uint8')
            mask = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask, [cnt], -1, (255), thickness=-1)

            # imgs = image[:, :, 0].astype('uint8')
            mask_rect = np.zeros(img.shape, np.uint8)
            # cv2.drawContours(mask_rect, [box], 0, (255), thickness=-1)
            subtracted = cv2.subtract(mask_rect, mask)
            ret2, th2 = cv2.threshold(subtracted, 100, 255, cv2.THRESH_BINARY)
            white = np.argwhere(th2 == 255)
            white = np.transpose(white)
            # cv2.imshow("mask",mask)
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

            dist1 = (xtop - y[1]) ** 2 + (ytop - y[0]) ** 2
            dist2 = (xbot - y[1]) ** 2 + (ybot - y[0]) ** 2
            if dist1 > dist2:

                cv2.line(image, (int(cx), int(cy)), (int(xtop), int(ytop)), (0, 0, 255), 3)
                angle = np.arctan2(ybot - cy, xbot - cx)
                # print(ybot - cy)
                # print(xbot - cx)
                xfront = xbot
                yfront = ybot
                xback = xtop
                yback = ytop
                coord = projection(np.array([xbot, cx, xtop]), np.array([ybot, cy, ytop]), cv_depth, drone_pose)
                cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
                cv2.circle(image,(int(xbot),int(ybot)),7,(255,0,127),-1)
                cv2.circle(image,(int(xtop),int(ytop)),7,(255,0,127),-1)
            else:
                cv2.line(image, (int(xbot), int(ybot)), (int(cx), int(cy)), (0, 0, 255), 3)

                angle = np.arctan2(ytop - cy, xtop - cx)
                # print(ybot - cy)
                # print(xbot - cx)
                xfront = xtop
                yfront = ytop
                xback = xbot
                yback = ybot
                coord = projection(np.array([xtop, cx, xbot]), np.array([ytop, cy, ybot]), cv_depth, drone_pose)
                cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
                cv2.circle(image,(int(xtop),int(ytop)),7,(255,0,127),-1)
                cv2.circle(image,(int(xbot),int(ybot)),7,(255,0,127),-1)

            # angle = angle * 180 / np.pi
            # x_y_yaw = str(cx) + "," + str(cy) + "," + str(angle)
            # cv2.putText(image, x_y_yaw, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # print("x, y, yaw")
            # print(cx, cy, angle)
            # cv2.imshow("image",image)
            # cv2.waitKey(1)

            result = [cx, cy, angle]
            # cv2.imshow("images", image)
            # cv2.waitKey(10)


            # ARCHIT CODE
            # print("Number of Points")
            # print(cx,cy,xfront,yfront,xback,yback)
            # cv2.circle(image,(int(cx),int(cy)),7,(255,0,127),-1)
            # cv2.circle(image,(int(xfront),int(yfront)),7,(255,0,127),-1)
            # cv2.circle(image,(int(xback),int(yback)),7,(255,0,127),-1)
            cv2.imshow("points",image)
            cv2.waitKey(1)
            # coord = projection(np.array([xfront, cx, xback]),np.array([yfront, cy, yback]),cv_depth, drone_pose)
            # coord_back = projection(xback,yback,cv_depth)
            # coord_front = projection(xfront,yfront,cv_depth)
            # print("Centre World Coordinates ---------- \n", coord)
            # print(coord.shape)

            # isMask = True
            if coord.shape[1] == 3:
                # isMask = True
                dZ = coord[0,2] - coord[2,2]
                dX = coord[0,0] - coord[2,0]

                yaw_angle = np.arctan2(dZ,dX)
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
                vel_x = vel_y = vel_z=0
                if xdata:
                    # print(xdata)
                    if time[-1] == header_im2.stamp.to_sec():
                        vel_x=xvel[-1]
                        vel_y=yvel[-1]
                        vel_z=zvel[-1]
                    else:
                        vel_x = (xdata[-1]-coord[0,2])/(time[-1]-header_im2.stamp.to_sec())
                        vel_y = (ydata[-1]-coord[1,2])/(time[-1]-header_im2.stamp.to_sec())
                        vel_z = (zdata[-1]-coord[2,2])/(time[-1]-header_im2.stamp.to_sec())

                # dist_temp = (xdata[-1]-coord_back[0])**2 + (ydata[-1]-coord_back[1])**2 + (zdata[-1]-coord_back[2])

                xdata.append(coord[0,2])
                ydata.append(coord[1,2])
                zdata.append(coord[2,2])
                time.append(header_im2.stamp.to_sec())
                xvel.append(vel_x)
                yvel.append(vel_y)
                zvel.append(vel_z)

                pubMsg.car_state.header = header_im2
                pubMsg.car_state.pose.pose.position.x = coord[0,2]
                pubMsg.car_state.pose.pose.position.y = coord[1, 2]
                pubMsg.car_state.pose.pose.position.z = coord[2, 2]

                # final_pose = geometry_msgs.msg.PoseStamped()
                # final_pose.pose.position.x = coord[0,2]
                # final_pose.pose.position.y = coord[1,2]
                # final_pose.pose.position.z = coord[2,2]
                # final_pose.header = drone_pose.header

                pubMsg.car_state.pose.pose.orientation.x = yaw_angle
                pubMsg.car_state.pose.pose.orientation.y = 0
                pubMsg.car_state.pose.pose.orientation.z = 0
                pubMsg.car_state.pose.pose.orientation.w = 0

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
                pub2.publish(bridge.cv2_to_imgmsg(mask))

                # pub4.publish(final_vel)
                # pub3.publish(isMask)
                # pub1.publish(final_pose)
                # pub2.publish(bridge.cv2_to_imgmsg(mask))



            # else:
            #     isMask = False

                # cv2.imshow("mask",mask)
                # cv2.waitKey(0)



        # else:
        #     isMask = False
        #     pub3.publish(isMask)
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
