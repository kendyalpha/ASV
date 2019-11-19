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


## 参数 ##
DBNAME = 'test.db'
num_thruster = 2

##


# connect to sqlite database
db_conn = sqlite3.connect(DBNAME)
# create a cursor to execute SQL commands
db_cursor = db_conn.cursor()

#  retrieve controller data from database
db_cursor.execute("SELECT * FROM PLAN")
plan_rows = db_cursor.fetchall()

# Save (commit) the changes
db_conn.commit()

# We can also close the connection if we are done with it.
# Just be sure any changes have been committed or they will be lost.
db_conn.close()

plandata = pd.DataFrame(plan_rows)
plandata.columns = ['ID', 'DATETIME', 'theta',
                    'kappa', 'lp_theta', 'lp_kappa', 'speed']
plandata['DATETIME'] = plandata['DATETIME'].astype(float)


# results of thrust on vessel
plt.figure(1, figsize=(12, 10))
plt.suptitle("Lattice Planner", fontsize=14)
plt.subplot(3, 1, 1)
plt.plot(plandata['theta'], '-r', lw=2)
plt.plot(plandata['lp_theta'], '--k', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('theta (deg)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(plandata['kappa'], '-r', lw=2)
plt.plot(plandata['lp_kappa'], '--k', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('kappa')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(plandata['speed'], '-r', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('speed (m/s)')
plt.grid(True)


plt.show()
