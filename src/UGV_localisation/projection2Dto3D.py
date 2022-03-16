import numpy as np

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