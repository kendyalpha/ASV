# /*
# ****************************************************************************
# * plotestimator.py:
# * Illustration of results of state estimator
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import math

# connect to sqlite database
db_conn = sqlite3.connect('dbtest.db')
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve GPS data from database
db_cursor.execute("SELECT * FROM GPS")
gps_rows = db_cursor.fetchall()

#  retrieve estimator data from database
db_cursor.execute("SELECT * FROM estimator")
estimator_rows = db_cursor.fetchall()


# Save (commit) the changes
db_conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
db_conn.close()


# convert lists to dataframe
gpsdata = pd.DataFrame(gps_rows)
gpsdata.columns = ['ID', 'DATETIME', 'UTC',
                   'latitude', 'longitude',
                   'heading', 'pitch', 'roll',
                   'altitude', 'Ve', 'Vn',
                   'roti', 'status', 'UTM_x', 'UTM_y']
gpsdata['DATETIME'] = gpsdata['DATETIME'].astype(float)

estimatordata = pd.DataFrame(estimator_rows)
estimatordata.columns = ['ID', 'DATETIME',
                         'meas_x', 'meas_y', 'meas_theta',
                         'meas_u', 'meas_v', 'meas_r',
                         'state_x', 'state_y', 'state_theta',
                         'state_u', 'state_v', 'state_r',
                         'perror_x', 'perror_y', 'perror_mz',
                         'verror_x', 'verror_y', 'verror_mz']
estimatordata['DATETIME'] = estimatordata['DATETIME'].astype(float)

timestamp = min(gpsdata["DATETIME"].loc[0], estimatordata["DATETIME"].loc[0])
gpsdata['DATETIME'] = (gpsdata['DATETIME']-timestamp)*86400
estimatordata['DATETIME'] = (estimatordata['DATETIME']-timestamp)*86400

plt.figure(1, figsize=(10, 8))
plt.suptitle("Plannar trajectory of GPS", fontsize=14)
plt.plot(gpsdata["UTM_x"], gpsdata["UTM_y"], '.r', lw=2, markersize=1)
plt.plot(estimatordata["meas_y"],
         estimatordata["meas_x"], 'ko', lw=2, markersize=1)

plt.axis('equal')
plt.xlabel('E (m)')
plt.ylabel('N (m)')
plt.legend(('GPS_UTM', 'COG'), loc='upper right')


plt.figure(1, figsize=(15, 10))
plt.suptitle("time series of motion", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(gpsdata['DATETIME'], gpsdata['UTM_y'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], estimatordata['meas_x'], '--k', lw=2)
plt.ylabel('X (m)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

plt.subplot(3, 1, 2)
plt.plot(gpsdata['DATETIME'], gpsdata['UTM_x'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], estimatordata['meas_y'], '--k', lw=2)
plt.ylabel('Y (m)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

plt.subplot(3, 1, 3)
plt.plot(gpsdata['DATETIME'], gpsdata['heading'], '-r', lw=2)
plt.plot(estimatordata['DATETIME'], (180/math.pi)
         * estimatordata['meas_theta'], '--k', lw=2)
plt.ylabel('theta (deg)')
plt.legend(('GPS Antenna', 'CoG'), loc='upper right')


plt.show()
