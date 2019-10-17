# /*
# ****************************************************************************
# * plotdubins.py:
# * Illustration of results of dubins path generator
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math


path = '../data/'
dubins = pd.read_csv(path+'dubins.csv')


plt.figure(1, figsize=(10, 8))
plt.suptitle("Dubins path", fontsize=14)
plt.plot(dubins['X'], dubins['Y'], '-r', lw=2)

length = 0.4
for index, row in dubins.iterrows():
    plt.arrow(row['X'], row['Y'], np.cos(row['theta'])*length,
              np.sin(row['theta'])*length, head_width=0.05, head_length=0.05)


plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.grid(True)


plt.show()
