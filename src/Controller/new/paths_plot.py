import numpy as np
import matplotlib.pyplot as plt
fig = plt.figure(figsize=(6.0,3.0),dpi = 600)
plt.rcParams['font.size'] = '8'
# ax = fig.gca()
plt.rcParams['ytick.left']=plt.rcParams['ytick.right']=True
plt.rcParams['xtick.top']=plt.rcParams['xtick.bottom']=True
plt.rcParams['ytick.direction']='in'
plt.rcParams['xtick.direction']='in'
plt.tick_params(axis='x', which='both')
# ax.tick_params(which="major", axis="x", direction="in")
# ax.tick_params(which="major", axis="y", direction="inout")

path1 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/new/world2.npy")[20:]
print(path1.shape)
plt.plot(path1[:,0],path1[:,1], label = "UGV Tracked Path | Phase 2", c = 'r', linewidth = 0.9, alpha = 0.8) 

path3 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/new/path_followed.npy")[20:630]

print(path1.shape)
plt.plot(path3[:,0],path3[:,1], c = 'r', linewidth = 0.9, alpha = 0.8) 


path2 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/src/Controller/new/world2_local_satwik.npy")[20:]
plt.plot(path2[:,0],path2[:,1], label="UAV Mapped Path | Phase 1" ,c = 'k', linewidth = 0.7, linestyle='dashed', alpha= 0.7) 
plt.legend(fontsize=8)
plt.grid()
plt.axes().set_aspect('equal')

plt.savefig("exp_cl.png",bbox_inches='tight', pad_inches = 0.03)
# plt.show()

# path3 = np.load("ugv_waypoints.npy")
# plt.plot(path3[:,0],path3[:,1]) 
# plt.legend("Sat old")
plt.show()