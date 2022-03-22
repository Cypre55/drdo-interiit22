from os import path
from xml.dom import xmlbuilder
import rospy
import numpy as np
from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler



f = open('cube.urdf', 'r')
model_xml = f.read()
path_file = np.load('world2_gazebo_rohitS.npy') # Name of path file
# path_file = path_file[100:, :]
print(path_file.shape)
reference_frame = ''                            # Change to 'iris' to plot the path in local iris frame. Iris frame not working tho.
rosns = rospy.get_namespace()
gzns = '/gazebo'

max_cnt = 0
for cnt in range(path_file.shape[0]/10):        # taking every 10th point. Adjust frequency based on number of points in path file
    model = 'cube' + str(cnt)                   # Change 'cube' to new name if you are plotting points in a world which already has a plot coz model name errors
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
        else:
            print("Cube{} spawned".format(cnt))
            max_cnt = cnt

