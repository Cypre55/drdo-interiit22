#!/usr/bin/python
from pyexpat import model
import rospy
from geometry_msgs.msg import Point,PoseStamped, Vector3
from gazebo_msgs.msg import ModelStates
import cv2 as cv
import tf
from tf.transformations import quaternion_matrix
import numpy as np
# from src.projection2Dto3D import projection

def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def dot(x,y):
    return np.sum(x*y)

drone_pose, model_pose, tgl = None, None, None

def projection():
    global drone_pose, model_pose, tgl

    if drone_pose is not None and model_pose is not None:
        quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
        mat = tf.transformations.quaternion_matrix(quaternion)
        tld=mat
        tld[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]

        quaternion = (model_pose.pose[1].orientation.x,model_pose.pose[1].orientation.y,model_pose.pose[1].orientation.z,model_pose.pose[1].orientation.w)
        mat = tf.transformations.quaternion_matrix(quaternion)
        tgd=mat
        tgd[:3,3]=[model_pose.pose[1].position.x, model_pose.pose[1].position.y, model_pose.pose[1].position.z]

        tld_inv = np.linalg.inv(tld)
        tgl = np.matmul(tgd, tld_inv)

def check():
    global drone_pose, model_pose, tgl
    if drone_pose is not None and model_pose is not None and tgl is not None:
        quaternion = (drone_pose.pose.orientation.x,drone_pose.pose.orientation.y,drone_pose.pose.orientation.z,drone_pose.pose.orientation.w)
        mat = tf.transformations.quaternion_matrix(quaternion)
        tlx=mat
        tlx[:3,3]=[drone_pose.pose.position.x, drone_pose.pose.position.y, drone_pose.pose.position.z]

        quaternion = (model_pose.pose[1].orientation.x,model_pose.pose[1].orientation.y,model_pose.pose[1].orientation.z,model_pose.pose[1].orientation.w)
        mat = tf.transformations.quaternion_matrix(quaternion)
        tgx=mat
        tgx[:3,3]=[model_pose.pose[1].position.x, model_pose.pose[1].position.y, model_pose.pose[1].position.z]

        lhs = np.matmul(tgl, tlx)
        rhs = tgx

        print("lhs", lhs.shape)
        print("rhs", rhs.shape)

        diff = np.sum(np.abs(lhs - rhs))
        print("diff", diff)

def transform1(data):
    global tgl
    data = data.T
    tgl_inv = np.linalg.inv(tgl)
    return np.matmul(tgl_inv, data)

def transform2(data):
    global tgl
    data = data.T
    return np.matmul(tgl, data)

def poseback(data):
    global drone_pose
    drone_pose=data

def modelback(data):
    global model_pose
    model_pose=data


def segmenter():
    rospy.init_node('segmenter')

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, poseback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, modelback)
    rate = rospy.Rate(10)
    # df = np.load("world1_gazebo_rohitS.npy")
    df = np.load("world2_gazebo_satwik.npy")
    df = np.hstack((df, np.ones((df.shape[0], 1))))
    print(df.shape)

    while not rospy.is_shutdown():
        projection()
        if tgl is not None:
            print("projected")
            df_local = transform1(df) # to calc from gazebo to local
            # df_gazebo = transform2(df) # to calc from local to gazebo
            print("saved")
            # np.save("world1_local_rohitS(1)", df_local.T[:,:3])
            np.save("world2_local_satwik.npy", df_local.T[:,:3])
            break

    rate.sleep()


if __name__ == '__main__':
    segmenter()