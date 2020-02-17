# /*
# ****************************************************************************
# * plotestimator.py:
# * Illustration of results of state estimator, using the estimator.db
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import db_parser
import matplotlib.pyplot as plt
import math

# parse data from database
measurement_data, state_data, error_data = db_parser.parse_estimator(
    '../../fileIO/recorder/data/estimator.db',
    '../../fileIO/recorder/config/dbconfig.json')

timestamp0 = min(measurement_data["DATETIME"].loc[0],
                 state_data["DATETIME"].loc[0],
                 error_data["DATETIME"].loc[0])
measurement_data['DATETIME'] = (measurement_data['DATETIME']-timestamp0)*86400
state_data['DATETIME'] = (state_data['DATETIME']-timestamp0)*86400
error_data['DATETIME'] = (error_data['DATETIME']-timestamp0)*86400

# plt.figure(1, figsize=(10, 8))
# plt.suptitle("Plannar trajectory of GPS", fontsize=14)
# plt.plot(gpsdata["UTM_x"], gpsdata["UTM_y"], '.r', lw=2, markersize=1)
# plt.plot(estimatordata["meas_y"],
#          estimatordata["meas_x"], 'ko', lw=2, markersize=1)

# plt.axis('equal')
# plt.xlabel('E (m)')
# plt.ylabel('N (m)')
# plt.legend(('GPS_UTM', 'COG'), loc='upper right')


# plt.figure(2, figsize=(15, 10))
# plt.suptitle("time series of motion", fontsize=12)
# plt.subplot(3, 1, 1)
# plt.plot(gpsdata['DATETIME'], gpsdata['UTM_y'], '-r', lw=2)
# plt.plot(estimatordata['DATETIME'], estimatordata['meas_x'], '--k', lw=2)
# plt.ylabel('X (m)')
# plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

# plt.subplot(3, 1, 2)
# plt.plot(gpsdata['DATETIME'], gpsdata['UTM_x'], '-r', lw=2)
# plt.plot(estimatordata['DATETIME'], estimatordata['meas_y'], '--k', lw=2)
# plt.ylabel('Y (m)')
# plt.legend(('GPS Antenna', 'CoG'), loc='upper right')

# plt.subplot(3, 1, 3)
# plt.plot(gpsdata['DATETIME'], gpsdata['heading'], '-r', lw=2)
# plt.plot(estimatordata['DATETIME'], (180/math.pi)
#          * estimatordata['meas_theta'], '--k', lw=2)
# plt.ylabel('theta (deg)')
# plt.legend(('GPS Antenna', 'CoG'), loc='upper right')


# plt.show()
