# /*
# ****************************************************************************
# * ReadRadarSpoke.py:
# * Illustration of results of Marine Radar Spoke
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

## 参数 ##
DBNAME = '../data/radar.db'

# connect to sqlite database
db_conn = sqlite3.connect(DBNAME)
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve controller data from database
db_cursor.execute("SELECT * FROM person")
radar_rows = db_cursor.fetchall()

db_conn.commit()
db_conn.close()

for row in radar_rows:
    title = "ID: " + str(row[0]) + "  azimuth: " + str(row[1])
    spokedata = []
    for i in row[3]:
        spokedata.append(i)

    samplerange = row[2]
    plt.figure(1, figsize=(20, 5))
    plt.plot(np.linspace(1, 512, 512)*samplerange, spokedata, '.', lw=2)
    plt.title(title)
    plt.xlabel('Distance(m)')
    plt.ylabel('strength')
    plt.grid(True)
    plt.pause(0.05)
    plt.clf()

plt.show()
