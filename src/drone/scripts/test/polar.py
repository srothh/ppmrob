import matplotlib.pyplot as plt
import numpy as np

#r = np.arange(0, 2, 0.01)
#theta = 2 * np.pi * r

azimut = 90

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_direction(-1)
ax.set_theta_offset(np.deg2rad(azimut)+np.pi/2)
ax.set_yticklabels([])
ax.set_rmax(1)
ax.set_xticks(np.arange(np.radians(0), np.radians(360), np.radians(10))) 
ax.set_xticklabels(np.arange(0,360,10), fontsize='small')
ax.grid(True)
ax.tick_params(labelsize=6)
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line

ax.arrow(0,0,np.deg2rad(0), 1, color='r')
ax.arrow(0,0,np.deg2rad(90), 1, color='g')
ax.arrow(0,0,np.deg2rad(270), 1, color='b')

plt.show()
