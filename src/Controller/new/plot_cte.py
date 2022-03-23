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

path1 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/cte2.npy")
path1 = path1[:1477]
print(path1.shape[0])
print(np.arange(10))
df = np.hstack((path1, np.arange(path1.shape[0])))
print(df.shape)
plt.plot(np.arange(path1.shape[0]), path1, label="Cross-Track Error | World 2" ,c = 'k', linewidth = 0.7, linestyle='dashed', alpha= 0.7) 
plt.legend(fontsize=8)
plt.grid()
plt.xlabel("iterations")
plt.ylabel("error (m)")
# plt.set_ylabel("ylabel")
# plt.axes().set_aspect('equal')

plt.savefig("e22_cte2.png",bbox_inches='tight', pad_inches = 0.03)


# path1 = np.load("/home/theabyss/interiit_new_ws/src/drdo_interiit22/cte1.npy")
# plt.plot(np.linspace(0, path1.shape[0], path1.shape[0]), path1)
plt.show()