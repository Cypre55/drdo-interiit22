import numpy as np
import matplotlib.pyplot as plt

path1 = np.load("path_followed.npy")
print(path1.shape)
plt.plot(path1[:,0],path1[:,1]) 
plt.legend("Sat new")

# path2 = np.load("path_new_rs.npy").T
# plt.plot(path2[:,0],path2[:,1]) 
# plt.legend("Ris")

path2 = np.load("sexy_path.npy").T[20:]
plt.plot(path2[:,0],path2[:,1]) 
plt.legend("Ris")


# path3 = np.load("ugv_waypoints.npy")
# plt.plot(path3[:,0],path3[:,1]) 
# plt.legend("Sat old")
plt.show()