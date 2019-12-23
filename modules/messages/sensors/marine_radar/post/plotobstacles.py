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


start_x = 3433823
start_y = 350891
end_x = 3433771
end_y = 351006


ox = [3433791, 3433791, 3433790, 3433789, 3433789,
      3433788, 3433787, 3433787, 3433786, 3433785]
oy = [350953, 350954, 350955, 350956, 350957,
      350958, 350959, 350959, 350959, 350960]

plt.figure(1, figsize=(12, 10))
plt.plot([start_y, end_y], [start_x, end_x], '-*r', lw=2)
plt.plot(oy, ox, 'bo')
plt.xlabel('y(m)')
plt.ylabel('x(m)')
plt.grid(True)


plt.show()
