# /*
# ***********************************************************************
# * realtime_Frenet_planner.py:
# * real time illustration of 2d vessel, static/dynamic obstacle
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


fig,  ((ax1, ax2)) = plt.subplots(
    nrows=1, ncols=2, figsize=(20, 12))
fig.suptitle('Simulation of Frenet Planner algorithm')


_vessel2d = vessel2d()

HOST = '127.0.0.1'
PORT = 9340                # 设置端口号

area = 30.0  # animation area length [m]
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(None)
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(1600)
        doubles_sequence = array.array('d', data)
        # vessel profile
        vessel2dnew = _vessel2d.perform_tran(
            doubles_sequence[1],  # x
            doubles_sequence[0],  # y
            doubles_sequence[2]  # heading
        )
        polygon = Polygon(vessel2dnew, True, color='black', alpha=0.4)
        ax1.add_patch(polygon)

        speed = doubles_sequence[3]  # speed
        N_ob = int(doubles_sequence[4])  # of obstacle
        N_bp = int(doubles_sequence[2 * N_ob + 5])

        # circles for obstacles
        x = np.zeros(N_ob)
        y = np.zeros(N_ob)
        for i in range(N_ob):
            x[i] = doubles_sequence[5+2*i]
            y[i] = doubles_sequence[6+2*i]

        # radii = 0.1*np.random.rand(N)
        radii = 0.2*np.ones(N_ob)
        for x1, y1, r in zip(x, y, radii):
            circle = Circle((y1, x1), r, color='red', alpha=0.4)
            ax1.add_patch(circle)

        # best path
        bestx = np.zeros(N_bp)
        besty = np.zeros(N_bp)
        bestspeed = np.zeros(N_bp)

        for i in range(N_bp):
            print(2 * N_ob + 3*i+8)
            bestx[i] = doubles_sequence[2 * N_ob + 3*i+6]
            besty[i] = doubles_sequence[2 * N_ob + 3*i+7]
            bestspeed[i] = doubles_sequence[2 * N_ob + 3*i+8]

        ax1.plot(besty[1:], bestx[1:], "-oc", markersize=3, alpha=0.4)

        ax1.axis('equal')
        ax1.set_xlim(doubles_sequence[1] - area, doubles_sequence[1] + area)
        ax1.set_ylim(doubles_sequence[0] - area, doubles_sequence[0] + area)
        ax1.set(xlabel='E (m)', ylabel='N (m)')

        ax2.plot(bestspeed)
        ax2.set(xlabel='Time (s)', ylabel='speed (m/s)')
        plt.pause(0.5)
        ax1.clear()
        ax2.clear()

    plt.show()
