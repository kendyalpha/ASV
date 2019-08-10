# /*
# ***********************************************************************
# * vessel2d.py:
# * generate
# * This header file can be read by C++ compilers
# *
# * by Hu.ZH(CrossOcean.ai)
# ***********************************************************************
# */


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection


class vessel2d:

    def __init__(self):

        # calc coefficient of quintic polynomial
        self.vesselprofile = np.array([[0, 5],
                                       [0.7, 3.8],
                                       [0.7, 0.6],
                                       [0.5, 0],
                                       [-0.5, 0],
                                       [-0.7, 0.6],
                                       [-0.7, 3.8]])

    def perform_tran(self, x_cog, y_cog, heading):
        trans = self.calc_transform(heading)
        vp = self.vesselprofile.dot(trans)
        vp[:, 0] = np.add(vp[:, 0], x_cog)
        vp[:, 1] = np.add(vp[:, 1], y_cog)
        return vp

    def calc_transform(self, heading):
        trans = np.array([[np.cos(heading), -np.sin(heading)],
                          [np.sin(heading), np.cos(heading)]])

        return trans


fig, ax = plt.subplots()
_vessel2d = vessel2d()
N = 5


for i in range(10):

    # circles
    x = np.random.rand(N)
    y = np.random.rand(N)
    radii = 0.1*np.random.rand(N)
    for x1, y1, r in zip(x, y, radii):
        circle = Circle((x1, y1), r, color='red', alpha=0.4)
        ax.add_patch(circle)

    # vessel profile
    angle = i*np.pi/4
    vessel2dnew = _vessel2d.perform_tran(1, 0, angle)
    polygon = Polygon(vessel2dnew, True, alpha=0.4)
    ax.add_patch(polygon)
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    plt.pause(0.05)
    ax.clear()

plt.show()
