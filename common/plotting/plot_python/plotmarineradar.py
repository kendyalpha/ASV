# /*
# ****************************************************************************
# * plotmarineradar.py:
# * Illustration of results of controller, using the controller.db
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import db_parser
import matplotlib.pyplot as plt
import math


radar_data = db_parser.parse_marineradar(
    '../../fileIO/recorder/data/marineradar.db',
    '../../fileIO/recorder/config/dbconfig.json', 512)

for i in radar_data['SpokeData']:
    print(i)
