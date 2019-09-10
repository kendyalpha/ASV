# /*
# ****************************************************************************
# * plotthrustallocation.py:
# * Illustration of results of thrust allocation
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math

path = '../data/'
Balpha = pandas.read_csv(path+'Balpha.csv')
alpha = pandas.read_csv(path+'alpha.csv')
u = pandas.read_csv(path+'u.csv')
alpha_deg = pandas.read_csv(path+'alpha_deg.csv')
dtau = pandas.read_csv(path+'tau.csv')
rotation = pandas.read_csv(path+'rotation.csv')

m = len(alpha.columns)


plt.figure(1, figsize=(8, 8))
plt.suptitle("Desired thrust and estimated force", fontsize=14)
plt.subplot(3, 1, 1)
plt.plot(Balpha['column 1'], '-r', lw=2)
plt.plot(dtau['column 1'], '--k', lw=2)
plt.xlabel('Sampling instant')
plt.ylabel('taux (N)')
plt.grid(True)
plt.legend(('estimated force', 'desired force'), loc='upper right')

plt.subplot(3, 1, 2)
plt.plot(Balpha['column 2'], '-r', lw=2)
plt.plot(dtau['column 2'], '--k', lw=2)
plt.xlabel('Sampling')
plt.ylabel('tauy (N)')
plt.grid(True)
plt.legend(('estimated force', 'desired force'), loc='upper right')

plt.subplot(3, 1, 3)
plt.plot(Balpha['column 3'], '-r', lw=2)
plt.plot(dtau['column 3'], '--k', lw=2)
plt.xlabel('Sampling')
plt.ylabel('taun (N*m)')
plt.grid(True)
plt.legend(('estimated force', 'desired force'), loc='upper right')

plt.figure(2, figsize=(8, 8))
plt.suptitle("thrust of each propeller", fontsize=14)
for i in range(m):
    plt.subplot(m, 1, i+1)
    plt.plot(u['column '+str(i+1)], lw=2)
    plt.xlabel('Sampling')
    plt.ylabel('u'+str(i+1))
    plt.grid(True)


plt.figure(3, figsize=(8, 8))
plt.suptitle("azimuth of each propeller", fontsize=14)
for i in range(m):
    plt.subplot(m, 1, i+1)
    plt.plot(alpha['column '+str(i+1)]*180/math.pi, '-r', lw=2)
    plt.plot(alpha_deg['column '+str(i+1)], ':k', lw=2)
    plt.legend(('double angle', 'int angle'), loc='upper right')
    plt.xlabel('Sampling')
    plt.ylabel('alpha(deg) '+str(i+1))
    plt.grid(True)


plt.figure(4, figsize=(8, 8))
plt.suptitle("rotation of each propeller", fontsize=14)
for i in range(m):
    plt.subplot(m, 1, i+1)
    plt.plot(rotation['column '+str(i+1)], lw=2)
    plt.xlabel('Sampling')
    plt.ylabel('n(rpm) '+str(i+1))
    plt.grid(True)

plt.show()
