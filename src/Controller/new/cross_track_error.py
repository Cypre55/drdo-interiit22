import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
path1 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/path_followed.npy")[20:]
path2 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/sexy_path.npy").T[20:,0:2]

# error_list = []
# for i in range(len(path1)):
    
#     x,y   = path1[:,0][i],path1[:,1][i]
#     print(x,y)
#     e = KDTree(path2).query(np.array([x,y]))[0]
#     error_list.append(e)

error_list = np.load("cte1.npy")

plt.show()