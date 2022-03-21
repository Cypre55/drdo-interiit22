import numpy as np
import matplotlib.pyplot as plt

path1 = np.load("Full_World1_RUN.npy")
plt.plot(path1[:,0],path1[:,1]) 
plt.legend("Sat new")

path2 = np.load("path_new_rs.npy")
plt.plot(path2[:,0],path2[:,1]) 
plt.legend("Ris")


path3 = np.load("ugv_waypoints.npy")
plt.plot(path3[:,0],path3[:,1]) 
plt.legend("Sat old")
plt.show()