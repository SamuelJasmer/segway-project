
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt



fig = plt.figure()
ax = fig.gca(projection='3d')

theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
z = [0,1]
x = [0,1]
y = [0,1]
ax.plot(x, y, z)
ax.legend()

plt.show()
