# /*
# ***********************************************************************
# * vessel2d.py:
# * real time illustration of 2d vessel, to demonstrate the
# * trajectory tracking algorithm
# *
# * by Hu.ZH(CrossOcean.ai)
# ***********************************************************************
# */


import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import socket
import array
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection


#
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


class sql:
    def __init__(self):
        self.dbpath = '../data/wp.db'

    def read2pandas(self):
        # connect to sqlite database
        db_conn = sqlite3.connect(self.dbpath)
        # create a cursor to execute SQL commands
        db_cursor = db_conn.cursor()

        #  retrieve GPS data from database
        db_cursor.execute("SELECT * FROM WP")
        wp_rows = db_cursor.fetchall()
        db_conn.commit()
        db_conn.close()
        wpdata = pd.DataFrame(wp_rows)
        wpdata.columns = ['ID', 'X', 'Y']

        return wpdata


_vessel2d = vessel2d()
_sql = sql()
wpdata = _sql.read2pandas()


fig,  ((ax1, ax2), (ax3, ax4)) = plt.subplots(
    nrows=2, ncols=2, figsize=(12, 12))
fig.suptitle('Simulation of trajectory tracking algorithm')

ax1.plot(wpdata['Y'], wpdata['X'], color='tab:gray', lineStyle='-', lw=1)
ax1.axis('equal')
ax1.set(xlabel='E (m)', ylabel='N (m)')

plt.show()


HOST = '127.0.0.1'
PORT = 9340                # 设置端口号

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(None)
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(160)
        doubles_sequence = array.array('d', data)

        # vessel profile
        vessel2dnew = _vessel2d.perform_tran(
            doubles_sequence[0],  # x
            doubles_sequence[1],  # y
            doubles_sequence[2]  # heading
        )
        polygon = Polygon(vessel2dnew, True, color='black', alpha=0.4)
        ax1.add_patch(polygon)

        # planner
        x = np.zeros(N_ob)
        y = np.zeros(N_ob)
        for i in range(N_ob):
            x[i] = doubles_sequence[3+2*i]
            y[i] = doubles_sequence[4+2*i]

        # trajectory tracking
        radii = 0.2*np.ones(N_ob)
        for x1, y1, r in zip(x, y, radii):
            circle = Circle((x1, y1), r, color='red', alpha=0.4)
            ax.add_patch(circle)

        # controller
        N_bp = int(doubles_sequence[13])
        bestx = np.zeros(N_bp)
        besty = np.zeros(N_bp)
        for i in range(N_bp):
            bestx[i] = doubles_sequence[14+2*i]
            besty[i] = doubles_sequence[15+2*i]
        plt.plot(bestx[1:], besty[1:], "-oc", markersize=3, alpha=0.4)

        plt.axis('equal')
        area = 15.0  # animation area length [m]
        plt.xlim(doubles_sequence[0] - area, doubles_sequence[0] + area)
        plt.ylim(doubles_sequence[1] - area, doubles_sequence[1] + area)
        # plt.axis([-52, 52, -50, 50])
        plt.pause(0.1)
        ax.clear()

    plt.show()
