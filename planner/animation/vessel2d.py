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
        self.vesselprofile = np.array([[2.5, 0],
                                       [1.9, -0.35],
                                       [0.3, -0.35],
                                       [0, -0.25],
                                       [0, 0.25],
                                       [0.3, 0.35],
                                       [1.9, 0.35]])

    def perform_tran(self, x_cog, y_cog, heading):
        trans = self.calc_transform(heading)
        vp = self.vesselprofile.dot(trans)
        vp[:, 0] = np.add(vp[:, 0], x_cog)
        vp[:, 1] = np.add(vp[:, 1], y_cog)
        return vp

    def calc_transform(self, heading):
        trans = np.array([[np.cos(heading), np.sin(heading)],
                          [-np.sin(heading), np.cos(heading)]])

        return trans


fig, ax = plt.subplots()
_vessel2d = vessel2d()
N_ob = 5
N_wp = 5
HOST = '127.0.0.1'
PORT = 9340                # 设置端口号

area = 15.0  # animation area length [m]
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(None)
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(800)
        doubles_sequence = array.array('d', data)

        # vessel profile
        vessel2dnew = _vessel2d.perform_tran(
            doubles_sequence[0],  # x
            doubles_sequence[1],  # y
            doubles_sequence[2]  # heading
        )
        polygon = Polygon(vessel2dnew, True, color='black', alpha=0.4)
        ax.add_patch(polygon)

        # circles for obstacles
        x = np.zeros(N_ob)
        y = np.zeros(N_ob)
        for i in range(N_ob):
            x[i] = doubles_sequence[3+2*i]
            y[i] = doubles_sequence[4+2*i]

        # radii = 0.1*np.random.rand(N)
        radii = 0.2*np.ones(N_ob)
        for x1, y1, r in zip(x, y, radii):
            circle = Circle((x1, y1), r, color='red', alpha=0.4)
            ax.add_patch(circle)

        # best path
        N_bp = int(doubles_sequence[13])
        bestx = np.zeros(N_bp)
        besty = np.zeros(N_bp)
        for i in range(N_bp):
            bestx[i] = doubles_sequence[14+2*i]
            besty[i] = doubles_sequence[15+2*i]
        plt.plot(bestx[1:], besty[1:], "-oc", markersize=3, alpha=0.4)

        plt.axis('equal')
        plt.xlim(doubles_sequence[0] - area, doubles_sequence[0] + area)
        plt.ylim(doubles_sequence[1] - area, doubles_sequence[1] + area)
        # plt.axis([-52, 52, -50, 50])
        plt.pause(0.1)
        ax.clear()

    plt.show()
