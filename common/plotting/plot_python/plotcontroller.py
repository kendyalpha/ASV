# /*
# ****************************************************************************
# * plotcontroller.py:
# * Illustration of results of controller, using the controller.db
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import db_parser
import matplotlib.pyplot as plt
import math


TA_data, setpoint_data = db_parser.parse_controller(
    '../../fileIO/recorder/data/controller.db',
    '../../fileIO/recorder/config/dbconfig.json', 4)

for i in TA_data['Rotation']:
    print(i)
