# /*
# ****************************************************************************
# * plotsimple1d.py:
# * Illustration of results of 1d simple spring
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
X = pandas.read_csv(path+'x.csv')

plt.figure(1, figsize=(8, 6))
plt.plot(X['column 2'], '-r', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('x')
plt.grid(True)
plt.show()
