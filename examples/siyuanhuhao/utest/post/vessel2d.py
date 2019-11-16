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
# wpdata = _sql.read2pandas()


fig,  ((ax1, ax2), (ax3, ax4)) = plt.subplots(
    nrows=2, ncols=2, figsize=(12, 12))
fig.suptitle('Simulation of trajectory tracking algorithm')

ax1.axis('equal')
ax1.set(xlabel='E (m)', ylabel='N (m)')


HOST = '127.0.0.1'
PORT = 9340                # 设置端口号
N_total = 100

# state
state_x = np.zeros(N_total)
state_y = np.zeros(N_total)
state_theta = np.zeros(N_total)
state_u = np.zeros(N_total)
state_v = np.zeros(N_total)
state_r = np.zeros(N_total)

# planner
planner_curvature = np.zeros(N_total)
planner_speed = np.zeros(N_total)
planner_wpx0 = np.zeros(N_total)
planner_wpy0 = np.zeros(N_total)
planner_wpx1 = np.zeros(N_total)
planner_wpy1 = np.zeros(N_total)

# controller
controller_taux = np.zeros(N_total)
controller_tauy = np.zeros(N_total)
controller_tauz = np.zeros(N_total)
controller_n1 = np.zeros(N_total)
controller_n2 = np.zeros(N_total)


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.settimeout(None)
    s.connect((HOST, PORT))
    while True:
        s.sendall(b'socket')
        data = s.recv(160)
        doubles_sequence = array.array('d', data)

        # update state
        state_x = np.delete(state_x, [0])
        state_y = np.delete(state_y, [0])
        state_theta = np.delete(state_theta, [0])
        state_u = np.delete(state_u, [0])
        state_v = np.delete(state_v, [0])
        state_r = np.delete(state_r, [0])

        state_x = np.append(state_x,  doubles_sequence[0])
        state_y = np.append(state_y,  doubles_sequence[1])
        state_theta = np.append(state_theta,  doubles_sequence[2])
        state_u = np.append(state_u,  doubles_sequence[3])
        state_v = np.append(state_v,  doubles_sequence[4])
        state_r = np.append(state_r,  doubles_sequence[5])

        # planner
        planner_curvature = np.delete(planner_curvature, [0])
        planner_speed = np.delete(planner_speed, [0])
        planner_wpx0 = np.delete(planner_wpx0, [0])
        planner_wpy0 = np.delete(planner_wpy0, [0])
        planner_wpx1 = np.delete(planner_wpx1, [0])
        planner_wpy1 = np.delete(planner_wpy1, [0])

        planner_curvature = np.append(planner_curvature,  doubles_sequence[6])
        planner_speed = np.append(planner_speed,  doubles_sequence[7])
        planner_wpx0 = np.append(planner_wpx0,  doubles_sequence[8])
        planner_wpy0 = np.append(planner_wpy0,  doubles_sequence[9])
        planner_wpx1 = np.append(planner_wpx1,  doubles_sequence[10])
        planner_wpy1 = np.append(planner_wpy1,  doubles_sequence[11])

        # controller
        controller_taux = np.delete(controller_taux, [0])
        controller_tauy = np.delete(controller_tauy, [0])
        controller_tauz = np.delete(controller_tauz, [0])
        controller_n1 = np.delete(controller_n1, [0])
        controller_n2 = np.delete(controller_n2, [0])

        controller_taux = np.append(controller_taux,  doubles_sequence[12])
        controller_tauy = np.append(controller_tauy,  doubles_sequence[13])
        controller_tauz = np.append(controller_tauz,  doubles_sequence[14])
        controller_n1 = np.append(controller_n1,  doubles_sequence[15])
        controller_n2 = np.append(controller_n2,  doubles_sequence[16])

        # vessel profile
        vessel2dnew = _vessel2d.perform_tran(
            doubles_sequence[1],  # x
            doubles_sequence[0],  # y
            doubles_sequence[2]  # heading
        )
        polygon = Polygon(vessel2dnew, True, color='black', alpha=0.4)
        ax1.add_patch(polygon)
        # ax1.plot(wpdata['Y'], wpdata['X'],
        #          color='tab:gray', lineStyle='-', lw=1)
        ax1.plot(doubles_sequence[9], doubles_sequence[8],
                 "oc", markersize=3, alpha=0.4)
        ax1.plot(doubles_sequence[11], doubles_sequence[10],
                 "oc", markersize=3, alpha=0.4)
        area = 15.0  # animation area length [m]
        ax1.set_xlim([doubles_sequence[1] - area, doubles_sequence[1] + area])
        ax1.set_ylim([doubles_sequence[0] - area, doubles_sequence[0] + area])

        ax2.plot(controller_n1, color='tab:gray',
                 lineStyle='-', lw=2, label='n1')
        ax2.plot(controller_n2, color='tab:blue',
                 lineStyle='--', lw=2, label='n2')
        ax2.legend()
        ax2.grid(True)

        ax3.plot(state_u, color='tab:gray',
                 lineStyle='-', lw=2, label='u')
        ax3.legend()
        ax3.grid(True)

        # plt.axis([-52, 52, -50, 50])
        plt.pause(0.1)
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

    plt.show()
