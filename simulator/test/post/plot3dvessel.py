# /*
# ****************************************************************************
# * plot3dvessel.py:
# * Illustration of results of 3d motion of vessel
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
X = pandas.read_csv(path+'x.csv')

plt.figure(1, figsize=(15, 10))
plt.suptitle("time series of motion", fontsize=12)
plt.subplot(3, 1, 1)
plt.plot(X['column 1'], '-r', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('x')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(X['column 2'], '-r', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('y')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(X['column 3'], '-r', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('theta')
plt.grid(True)

plt.show()
