# /*
# ****************************************************************************
# * plotspline.py:
# * Illustration of results of Polynomial interpolation using spline
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
X = pandas.read_csv(path+'x.csv')
Y = pandas.read_csv(path+'y.csv')
spline_X = pandas.read_csv(path+'spline_x.csv')
spline_Y = pandas.read_csv(path+'spline_y.csv')


plt.figure(1, figsize=(8, 6))
plt.suptitle("Polynomial interpolation using spline", fontsize=13)
plt.plot(X, Y, '+', lw=2, markersize=13)
plt.plot(spline_X, spline_Y, '--k', lw=2)
plt.show()
