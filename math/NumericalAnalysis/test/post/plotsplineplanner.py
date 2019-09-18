# /*
# ****************************************************************************
# * plotsplineplanner.py:
# * Illustration of results of 2d spline
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
s = pandas.read_csv(path+'s.csv')
X = pandas.read_csv(path+'x.csv')
Y = pandas.read_csv(path+'y.csv')
rx = pandas.read_csv(path+'rx.csv')
ry = pandas.read_csv(path+'ry.csv')
k = pandas.read_csv(path+'k.csv')
dk = pandas.read_csv(path+'dk.csv')
yaw = pandas.read_csv(path+'yaw.csv')

plt.figure(1, figsize=(8, 6))
plt.suptitle("Polynomial interpolation using spline", fontsize=13)
plt.plot(X, Y, '+', lw=2, markersize=13)
plt.plot(rx, ry, '--k', lw=2)
plt.grid(True)
plt.axis('equal')


plt.figure(2, figsize=(8, 6))
plt.suptitle("cuvature", fontsize=13)
plt.plot(s, k, lw=2)
plt.grid(True)

plt.figure(3, figsize=(8, 6))
plt.suptitle("yaw", fontsize=13)
plt.plot(s, yaw, lw=2)
plt.grid(True)

plt.figure(4, figsize=(8, 6))
plt.suptitle("d k", fontsize=13)
plt.plot(s, dk, lw=2)
plt.grid(True)
plt.show()
