# /*
# ***********************************************************************
# * vessel2d.py:
# * real time illustration of 2d vessel, to demonstrate the
# * trajectory tracking algorithm
# *
# * by Hu.ZH(CrossOcean.ai)
# ***********************************************************************
# */


import numpy as np
import matplotlib.pyplot as plt
import socket
import array
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection


class vessel2d:

    def __init__(self):

        # calc coefficient of quintic polynomial
        self.vesselprofile = np.array([[0, 1.5],
                                       [0.5, 1.0],
                                       [0.5, -1.4],
                                       [0.4, -1.6],
                                       [-0.4, -1.6],
                                       [-0.5, -1.4],
                                       [-0.5, 1.0]])

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

# vessel profile
vessel2dnew = _vessel2d.perform_tran(
    0,  # x
    0,  # y
    np.pi/4  # heading
)
polygon = Polygon(vessel2dnew, True, color='black', alpha=0.4)
ax.add_patch(polygon)
plt.axis('equal')

plt.show()
