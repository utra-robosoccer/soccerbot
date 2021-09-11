import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize


ph = np.linspace(0, 2*np.pi, 13)
x = np.cos(ph)
y = np.sin(ph)
u = np.cos(ph)
v = np.sin(ph)
colors = np.arctan2(u, v)

norm = Normalize()
norm.autoscale(colors)
# we need to normalize our colors array to match it colormap domain
# which is [0, 1]

colormap = cm.inferno
# pick your colormap here, refer to
# http://matplotlib.org/examples/color/colormaps_reference.html
# and
# http://matplotlib.org/users/colormaps.html
# for details
plt.figure(figsize=(6, 6))
plt.xlim(-2, 2)
plt.ylim(-2, 2)
plt.quiver(x, y, u, v, color=colormap(norm(colors)),  angles='xy',
           scale_units='xy', scale=1, pivot='mid')
plt.show()