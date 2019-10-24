# /*
# ****************************************************************************
# * plotKalman.py:
# * Illustration of results of state estimator
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

# connect to sqlite database
db_conn = sqlite3.connect('../data/Thu Oct 24 10:10:06 2019.db')
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve estimator data from database
db_cursor.execute("SELECT * FROM estimator")
estimator_rows = db_cursor.fetchall()

# Save (commit) the changes
db_conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
db_conn.close()


# convert lists to dataframe
estimatordata = pd.DataFrame(estimator_rows)
estimatordata.columns = ['ID', 'DATETIME',
                         'meas_x', 'meas_y', 'meas_theta',
                         'meas_u', 'meas_v', 'meas_r',
                         'state_x', 'state_y', 'state_theta',
                         'state_u', 'state_v', 'state_r',
                         'perror_x', 'perror_y', 'perror_mz',
                         'verror_x', 'verror_y', 'verror_mz']
estimatordata['DATETIME'] = estimatordata['DATETIME'].astype(float)


# convert datetime to second
estimatordata['DATETIME'] = (
    estimatordata['DATETIME']-estimatordata["DATETIME"].iloc[0])*86400


# 选择显示的时间段
time_stamp_select = np.array([50, 350])

estimatordata = estimatordata[(estimatordata['DATETIME'] > time_stamp_select[0])
                              & (estimatordata['DATETIME'] < time_stamp_select[1])]


plt.figure(1, figsize=(10, 8))
plt.suptitle("Plannar trajectory of estimator", fontsize=14)
plt.plot(estimatordata["state_y"], estimatordata["state_x"],
         ':k', lw=2, markersize=1)
plt.plot(estimatordata["meas_y"],
         estimatordata["meas_x"], '.r', lw=2, markersize=1)

plt.axis('equal')
plt.xlabel('E (m)')
plt.ylabel('N (m)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.figure(2, figsize=(15, 10))
plt.suptitle("time series of motion", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(estimatordata['DATETIME'], estimatordata['state_x'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_x'], '--k', lw=2)
plt.ylabel('X (m)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.subplot(3, 1, 2)
plt.plot(estimatordata['DATETIME'], estimatordata['state_y'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_y'], '--k', lw=2)
plt.ylabel('Y (m)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.subplot(3, 1, 3)
plt.plot(estimatordata['DATETIME'], estimatordata['state_theta'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], estimatordata['meas_theta'], '--k', lw=2)
plt.ylabel('theta (rad)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.figure(3, figsize=(15, 10))
plt.suptitle("time series of velocity", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(estimatordata['DATETIME'], estimatordata['state_u'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_u'], '--k', lw=2)
plt.ylabel('u (m/s)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.subplot(3, 1, 2)
plt.plot(estimatordata['DATETIME'], estimatordata['state_v'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_v'], '--k', lw=2)
plt.ylabel('v (m/s)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.subplot(3, 1, 3)
plt.plot(estimatordata['DATETIME'], estimatordata['state_r'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], estimatordata['meas_r'], '--k', lw=2)
plt.ylabel('r (rad/s)')
plt.legend(('state', 'measurement'), loc='upper right')


plt.show()
