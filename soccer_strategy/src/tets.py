
import numpy as np
from vispy.plot import Fig

fig = Fig()
ax_left = fig[0, 0]

data = [[0,2],[1,4]]
ax_left.plot(data)
data = [[0,0],[1,3]]
ax_left.plot(data)
data = [[0,1],[1,5]]
ax_left.plot(data)
fig.show(run=True)
