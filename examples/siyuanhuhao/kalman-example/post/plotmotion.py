# /*
# ****************************************************************************
# * plottrajectorytracking.py:
# * Illustration of results of trajectory tracking results
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
db_conn = sqlite3.connect('../data/Mon Oct 21 09:44:49 2019.db')
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve gps data from database
db_cursor.execute("SELECT * FROM GPS")
gps_rows = db_cursor.fetchall()

#  retrieve estimator data from database
db_cursor.execute("SELECT * FROM estimator")
estimator_rows = db_cursor.fetchall()

#  retrieve planner data from database
db_cursor.execute("SELECT * FROM stm32")
stm32_rows = db_cursor.fetchall()

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


gpsdata = pd.DataFrame(gps_rows)
gpsdata.columns = ['ID', 'DATETIME',
                   'UTC', 'latitude', 'longitude',
                   'heading', 'pitch', 'roll',
                   'altitude', 'Ve', 'Vn',
                   'roti', 'status', 'UTM_x', 'UTM_y']
gpsdata['DATETIME'] = gpsdata['DATETIME'].astype(float)


stm32data = pd.DataFrame(stm32_rows)
stm32data.columns = ['ID', 'DATETIME',
                     'stm32_link', 'stm32_status', 'command_u1',
                     'command_u2', 'feedback_u1', 'feedback_u2',
                     'feedback_pwm1', 'feedback_pwm2',
                     'RC_X', 'RC_Y', 'RC_Mz', 'voltage_b1',
                     'voltage_b2', 'voltage_b3']
stm32data['DATETIME'] = stm32data['DATETIME'].astype(float)

# convert datetime to second
timestamp = min(estimatordata["DATETIME"].loc[0],
                gpsdata["DATETIME"].loc[0],
                stm32data["DATETIME"].loc[0])
gpsdata['DATETIME'] = (gpsdata['DATETIME']-timestamp)*86400
estimatordata['DATETIME'] = (estimatordata['DATETIME']-timestamp)*86400
stm32data['DATETIME'] = (stm32data['DATETIME']-timestamp)*86400

# 选择显示的时间段
time_stamp_select = np.array([1, 400])

estimatordata = estimatordata[(estimatordata['DATETIME'] > time_stamp_select[0])
                              & (estimatordata['DATETIME'] < time_stamp_select[1])]
gpsdata = gpsdata[(gpsdata['DATETIME'] > time_stamp_select[0])
                  & (gpsdata['DATETIME'] < time_stamp_select[1])]

stm32data = stm32data[(stm32data['DATETIME'] > time_stamp_select[0])
                      & (stm32data['DATETIME'] < time_stamp_select[1])]

# 计算协方差矩阵
state_data = estimatordata.iloc[:, 2:7]
gps_data = gpsdata.iloc[:, 11:12]*math.pi/180/60
# state_data = state_data.join(gpsdata['roti']*math.pi/180/60)
# print(state_data)
print(state_data.cov())
print(gps_data.cov())

plt.figure(1, figsize=(10, 8))
plt.suptitle("Plannar trajectory of GPS", fontsize=14)
plt.plot(gpsdata["UTM_x"], gpsdata["UTM_y"], '.r', lw=2, markersize=1)
plt.plot(estimatordata["meas_y"],
         estimatordata["meas_x"], 'ko', lw=2, markersize=1)

plt.axis('equal')
plt.xlabel('E (m)')
plt.ylabel('N (m)')
plt.legend(('GPS_UTM', 'COG'), loc='upper right')


plt.figure(2, figsize=(15, 10))
plt.suptitle("time series of motion", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(gpsdata['DATETIME'], gpsdata['UTM_y'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_x'], '--k', lw=2)
plt.ylabel('X (m)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

plt.subplot(3, 1, 2)
plt.plot(gpsdata['DATETIME'], gpsdata['UTM_x'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_y'], '--k', lw=2)
plt.ylabel('Y (m)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

plt.subplot(3, 1, 3)
plt.plot(gpsdata['DATETIME'], gpsdata['heading'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], (180/math.pi)
         * estimatordata['meas_theta'], '--k', lw=2)
plt.ylabel('theta (deg)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')


plt.figure(3, figsize=(15, 10))
plt.suptitle("time series of velocity", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_u'], '--k', lw=2)
plt.ylabel('u (m/s)')

plt.subplot(3, 1, 2)
plt.plot(estimatordata['DATETIME'],
         estimatordata['meas_v'], '--k', lw=2)
plt.ylabel('v (m/s)')

plt.subplot(3, 1, 3)
plt.plot(estimatordata['DATETIME'], estimatordata['meas_r'], '--k', lw=2)
plt.ylabel('r (deg/s)')
plt.show()
