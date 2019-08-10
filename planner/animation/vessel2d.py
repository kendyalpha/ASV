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
import socket
import array
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
HOST = '127.0.0.1'
PORT = 9340                # 设置端口号


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(104)
        doubles_sequence = array.array('d', data)
        print('Received', doubles_sequence[1])

        # vessel profile
        vessel2dnew = _vessel2d.perform_tran(
            doubles_sequence[0],  # x
            doubles_sequence[1],  # y
            doubles_sequence[2]  # heading
        )
        polygon = Polygon(vessel2dnew, True, alpha=0.4)
        ax.add_patch(polygon)

        # circles
        x = np.zeros(N)
        y = np.zeros(N)
        for i in range(N):
            x[i] = doubles_sequence[3+2*i]
            y[i] = doubles_sequence[4+2*i]

        radii = 0.1*np.random.rand(N)
        for x1, y1, r in zip(x, y, radii):
            circle = Circle((x1, y1), r, color='red', alpha=0.4)
            ax.add_patch(circle)

        plt.axis('equal')
        plt.axis([-12, 12, -10, 10])
        plt.pause(0.05)
        ax.clear()

    plt.show()
