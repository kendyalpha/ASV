# /*
# ****************************************************************************
# * plotminiball.py:
# * Illustration of results of smallest closing ball of points
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import pandas
import matplotlib.pyplot as plt
import math


path = '../data/'
Points = pandas.read_csv(path+'points.csv')
circle_data = pandas.read_csv(path+'circle.csv')
circle_data = circle_data['column 1']


plt.figure(1, figsize=(8, 8))
plt.title("Smallest Closing Ball of Points", fontsize=14)
plt.plot(Points['column 1'], Points['column 2'], '.b')
circle = plt.Circle([circle_data[0], circle_data[1]], circle_data[2],
                    facecolor='none', edgecolor='r')
plt.gca().add_patch(circle)

plt.axis('scaled')
plt.show()
