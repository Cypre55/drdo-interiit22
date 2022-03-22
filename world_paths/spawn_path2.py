from os import path
from xml.dom import xmlbuilder
import rospy
import numpy as np
from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
import tf
from tf.transformations import quaternion_from_euler

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

def transform(data):
    global tgl
    data = data.T
    tgl_inv = np.linalg.inv(tgl)
    return np.matmul(tgl_inv, data)

def transforminv(data):
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
    path_file_d = np.load('path_taken_world1_gazebo_coord.npy')
    path_file_d = np.hstack((path_file_d, np.ones((path_file_d.shape[0], 1))))
    print(path_file_d.shape)
    # while not rospy.is_shutdown():
    #     # print("reached")
    #     # print(tgl)
    #     projection()
    #     if tgl is not None:
    #         print("projected")
    #         break

    # print("hello")
    # rate.sleep()

    f = open('cube.urdf', 'r')
    model_xml = f.read()
    reference_frame = 'iris'
    rosns = rospy.get_namespace()
    gzns = '/gazebo'
    max_cnt = 0

    # path_file = transform(path_file_d).T
    path_file = path_file_d
    # path_file_gz = path_file
    for cnt in range(path_file.shape[0]/10):
        model = 'cube' + str(cnt)
        if cnt*10 < path_file.shape[0]:
            x = path_file[cnt*10, 0]
            y = path_file[cnt*10, 1]
            z = path_file[cnt*10, 2] + 2
            initial_pose = Pose()
            initial_pose.position.x = x
            initial_pose.position.y = y
            initial_pose.position.z = z
            q = quaternion_from_euler(0, 0, 0)
            initial_pose.orientation = Quaternion(*q)

            success = gazebo_interface.spawn_urdf_model_client(model, model_xml, rosns,
                                                                    initial_pose, reference_frame,
                                                                    gzns)
            if not success:
                print("Error, couldn't spawn cube{}".format(cnt))
                # break
            else:
                print("Cube{} spawned".format(cnt))
                max_cnt = cnt



if __name__ == '__main__':
    segmenter()

