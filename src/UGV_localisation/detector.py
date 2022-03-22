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
    # print("rgb CALLED")

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    global imgimg, header_imgimg
    header_imgimg = data.header
    imgimg = cv_image


def depthCallback(data):
    # print("DEPTH CALLED")
    bridge = CvBridge()
    try:
        cv_depth = bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
        print(e)
    global im2, header_im2
    header_im2 = data.header
    im2 = cv_depth


def poseCallback(data):
    # print("pose CALLED")

    global drone_pose
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



def detector():
    rospy.init_node('detector')
    global imgimg, im2, drone_pose, cv_mask
    imgimg = None
    im2 = None
    drone_pose = None
    cv_mask = None
    # print("SUBSCRIBERS CALLED")
    rospy.Subscriber("/depth_camera/depth/image_raw", Image, depthCallback)
    rospy.Subscriber("/depth_camera/rgb/image_raw", Image, rgbCallback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseCallback)
    rospy.Subscriber("/lane/mask", Image, maskCallback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if imgimg is not None and im2 is not None and drone_pose is not None and cv_mask is not None:
            calculations()
            imgimg = None
            im2 = None
            drone_pose = None
            cv_mask = None
        rate.sleep()


def projection(cx, cy, img,
               K=None):  # takes 2 arrays of dim 1xn of x and y pixel coordinates and returns local 3d coordinates
    # Inputs:
    # global quaternion,transform_mat,transform_mat_c_d
    # cx, cy: Points in Pixel coordinates
    # print("projection CALLED")

    cx = cx.reshape(-1)
    cy = cy.reshape(-1)
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

    k_int = np.array([[554.254691191187, 0.0, 320.5],
                      [0.0, 554.254691191187, 240.5],
                      [0.0, 0.0, 1.0]])
    X = np.matmul(np.linalg.inv(k_int), X)
    unit_vec = X / np.linalg.norm(X, axis=0)
    dep = img[cx, cy]
    new_vec = unit_vec * dep

    global drone_pose
    quaternion = (drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z,
                  drone_pose.pose.orientation.w)
    mat = tf.transformations.quaternion_matrix(quaternion)
    transform_mat = mat
    transform_mat[:3, 3] = [drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]
    new_vec_p = np.vstack((new_vec, np.ones(new_vec.shape[1])))
    transform_mat_c_d = np.array([[0., -1, 0., 0.],
                                  [-1., 0., 0., 0.],
                                  [0., 0., -1., 0.],
                                  [0., 0., 0., 1.]])
    coord = np.matmul(transform_mat_c_d, new_vec_p)
    coord = np.matmul(transform_mat, coord)
    return coord


def calculations():
    if (len(xdata) == 3):
        xdata.pop(0)
    if (len(ydata) == 3):
        ydata.pop(0)
    if (len(zdata) == 3):
        zdata.pop(0)
    if(len(time)==3):
        time.pop(0)
    if (len(xvel) == 3):
        xvel.pop(0)
    if (len(yvel) == 3):
        yvel.pop(0)
    if (len(zvel) == 3):
        zvel.pop(0)
    # print("calculations called")
    bridge = CvBridge()
    global imgimg, im2, drone_pose, header_im2, header_imgimg, ellipse
    image = imgimg
    cv_depth = im2
    font = cv2.FONT_HERSHEY_SIMPLEX
    # cv2.imshow("input",image)
    # cv2.waitKey(1)
    # cv2.imwrite("white_overlay.png",image)


    blur = cv2.medianBlur(image, 7)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    gray = cv2.bitwise_not(gray)

    # kernel = np.ones((3,3), np.uint8)
    # cv_mask_dil = cv2.dilate(cv_mask, kernel, iterations=1)
    thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)[1]
    # thresh = 255 - cv_mask_dil
    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 400
    max_area = 40000

    pubMsg = customMessage()
    pubMsg.header = header_imgimg
    pubMsg.isMaskDetected.data = False
    pubMsg.isCarNinety.data = False

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
        ratioE = 0
        if (approx.shape[0] >= 5):
            (x, y), (MA, ma), angle = cv2.fitEllipse(approx)

            ellipse = cv2.fitEllipse(approx)
            x = int(x)
            y = int(y)
            if MA==0:
                condition = False
            MA = max(MA,1e-6)
            ratioE = ma/MA
            condition = ratioE>1.5 and ratioE<2.4
            # print(ratioE)
        else:
            condition = True

        if  area_ratio>0.8 and area > min_area and area < max_area and condition: #area_ratio > 0.8 and
            pubMsg.isMaskDetected.data = True
            # cv2.drawContours(image, [box],0,(200,0,0),3,)
            cv2.drawContours(image, [approx], 0, (0, 200, 0), 2 )
            # cv2.drawContours(image, [ellipse], 0, (200, 0, 0), 2 )
            
            if ratioE != 0:
                image = cv2.ellipse(image,ellipse,(0,255,0),2)
                print(ratioE)
                
                cv2.putText(image, '{}'.format(ratioE), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            cv2.circle(image, (int(cx), int(cy)), 7, (0, 0, 255), -1)
            # CREATING MASK OF CONTOURx
            img = image[:, :, 0].astype('uint8')
            mask = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask, [cnt], -1, (255), thickness=-1)

            mask_rect = np.zeros(img.shape, np.uint8)
            cv2.drawContours(mask_rect, [box], 0, (255), thickness=-1)
            subtracted = cv2.subtract(mask_rect, mask)
            ret2, th2 = cv2.threshold(subtracted, 100, 255, cv2.THRESH_BINARY)
            # cv2.imshow("subtracted",subtracted)
            # cv2.waitKey(1)
            # subtracted = cv2.subtract(mask_rect,th2)
            white = np.argwhere(th2 == 255)
            white = np.transpose(white)
            y = np.mean(white, axis=1)
            # cv2.circle(image, (int(y[1]), int(y[0])), 7, (0, 255, 255), -1)
            dist1 = (box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2
            dist2 = (box[1][0] - box[2][0]) ** 2 + (box[1][1] - box[2][1]) ** 2
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

                # cv2.line(image, (int(cx), int(cy)), (int(xtop), int(ytop)), (0, 0, 255), 3)
                xfront = xbot
                yfront = ybot
                xback = xtop
                yback = ytop
            else:
                # cv2.line(image, (int(xbot), int(ybot)), (int(cx), int(cy)), (0, 0, 255), 3)
                xfront = xtop
                yfront = ytop
                xback = xbot
                yback = ybot
            if cx>32 and cx<608 and cy>24 and cy<456:
                pubMsg.isCarNinety.data = True


            coord = projection(np.array([yfront,cy,yback]), np.array([xfront,cx,xback]), cv_depth, drone_pose)
            # cv2.circle(image,(int(cx),int(cy)),7,(0,0,255),-1)
            cv2.circle(image, (int(xfront), int(yfront)), 7, (0, 0,255), -1)
            # cv2.arrowedLine(image,(int(cx),int(cy)),(int(xfront), int(yfront)),(0,255,0),thickness = 4)

            cv2.imshow("images", image)
            cv2.waitKey(1)
            pub2.publish(bridge.cv2_to_imgmsg(image))

            if coord.shape[1] == 3:
                # isMask = True
                dY = coord[1, 0] - coord[1, 2]
                dX = coord[0, 0] - coord[0, 2]

                yaw_angle = np.arctan2(dY, dX)

                vel_x = vel_y = vel_z = 0
                if len(xdata) != 0:
                    dT = header_im2.stamp.to_sec() - time[-1]
                    dT = max(dT, 1e-3)
                    vel_x = (coord[0, 2] - xdata[-1]) / (dT)
                    vel_y = (coord[1, 2] - ydata[-1]) / (dT)
                    vel_z = (coord[2, 2] - zdata[-1]) / (dT)
                else:
                    vel_x = 0
                    vel_y = 0
                    vel_z = 0

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

                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)

                pubMsg.car_state.pose.pose.orientation.x = quaternion[0]
                pubMsg.car_state.pose.pose.orientation.y = quaternion[1]
                pubMsg.car_state.pose.pose.orientation.z = quaternion[2]
                pubMsg.car_state.pose.pose.orientation.w = quaternion[3]

                pubMsg.car_state.twist.twist.linear.x = vel_x
                pubMsg.car_state.twist.twist.linear.y = vel_y
                pubMsg.car_state.twist.twist.linear.z = vel_z

                pubMsg.car_state.twist.twist.angular.x = 0
                pubMsg.car_state.twist.twist.angular.y = 0
                pubMsg.car_state.twist.twist.angular.z = 0



                pub1.publish(pubMsg)

        else:
            # isMask = False
            pub1.publish(pubMsg)

if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass
