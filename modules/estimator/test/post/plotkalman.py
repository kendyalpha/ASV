# /*
# ****************************************************************************
# * plotkalman.py:
# * Illustration of Kalman filtering
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
kalman = pandas.read_csv(path+'kalman.csv')
observed = pandas.read_csv(path+'observed.csv')
truex = pandas.read_csv(path+'truex.csv')
Peigen = pandas.read_csv(path+'EigenP.csv')

plt.figure(1, figsize=(16, 7))
plt.suptitle("Kalman filtering", fontsize=14)
plt.subplot(1, 2, 1)
plt.plot(kalman['column 1'], '-r', lw=2)
plt.plot(observed['column 1'], '--k', lw=2)
plt.legend(('Kalman', 'observed'), loc='upper right')

plt.subplot(1, 2, 2)
plt.plot(Peigen['column 1'], '-r', lw=2)
plt.show()
