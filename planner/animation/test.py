import numpy as np
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


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
        self.vesselprofile[:, 0] = np.add(vp[:, 0], x_cog)
        self.vesselprofile[:, 1] = np.add(vp[:, 1], y_cog)
        return self.vesselprofile

    def calc_transform(self, heading):
        trans = np.array([[np.cos(heading), -np.sin(heading)],
                          [np.sin(heading), np.cos(heading)]])

        return trans


# Fixing random state for reproducibility
np.random.seed(19680801)


fig, ax = plt.subplots()

N = 5
x = np.random.rand(N)
y = np.random.rand(N)
radii = 0.1*np.random.rand(N)
patches = []
for x1, y1, r in zip(x, y, radii):
    circle = Circle((x1, y1), r, color='red', alpha=0.4)
    ax.add_patch(circle)


angle = np.pi/2

_vessel2d = vessel2d()
vessel2dnew = _vessel2d.perform_tran(1, 0, angle)

polygon = Polygon(vessel2dnew, True, alpha=0.4)
ax.add_patch(polygon)


# plt.axis([xmin, xmax, ymin, ymax])
plt.axis('auto')
plt.axis('equal')
plt.show()
