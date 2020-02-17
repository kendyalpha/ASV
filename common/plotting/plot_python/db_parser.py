# /*
# ****************************************************************************
# * db_parser.py:
# * Illustration of results of state estimator, using the estimator.db
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */

import struct
import sqlite3
import json
import pandas as pd


def parse_controller(db_path, db_config_path, num_thruster):
    # Read the db config file
    with open(db_config_path) as f:
        dbconfig = json.load(f)

    # connect to sqlite database
    db_conn = sqlite3.connect(db_path)
    # create a cursor to execute SQL commands
    db_cursor = db_conn.cursor()

    #  retrieve data from database
    db_cursor.execute("SELECT * FROM TA")
    TA_rows = db_cursor.fetchall()
    db_cursor.execute("SELECT * FROM setpoint")
    setpoint_rows = db_cursor.fetchall()

    db_conn.commit()
    db_conn.close()

    # thrust allocation dataframe
    TA_items = ['ID', 'DATETIME']
    for i in dbconfig['controller']['TA']:
        TA_items.append(i[0])
    TA_data = pd.DataFrame(TA_rows)
    TA_data.columns = TA_items
    TA_data['DATETIME'] = TA_data['DATETIME'].astype(float)

    # parse the BLOB data
    unpack_formats = '=' + str(num_thruster) + 'i'
    azimuth = []
    rotation = []
    for onerow in TA_data['Azimuth']:
        azimuth.append(struct.unpack(unpack_formats, onerow))
    for onerow in TA_data['Rotation']:
        rotation.append(struct.unpack(unpack_formats, onerow))

    TA_data['Azimuth'] = azimuth
    TA_data['Rotation'] = rotation

    # setpoint dataframe
    setpoint_items = ['ID', 'DATETIME']
    for i in dbconfig['controller']['setpoint']:
        setpoint_items.append(i[0])
    setpoint_data = pd.DataFrame(setpoint_rows)
    setpoint_data.columns = setpoint_items
    setpoint_data['DATETIME'] = setpoint_data['DATETIME'].astype(float)

    return TA_data, setpoint_data


def parse_estimator(db_path, db_config_path):
    # Read the db config file
    with open(db_config_path) as f:
        dbconfig = json.load(f)

    # connect to sqlite database
    db_conn = sqlite3.connect(db_path)
    # create a cursor to execute SQL commands
    db_cursor = db_conn.cursor()

    #  retrieve data from database
    db_cursor.execute("SELECT * FROM measurement")
    measurement_rows = db_cursor.fetchall()
    db_cursor.execute("SELECT * FROM state")
    state_rows = db_cursor.fetchall()
    db_cursor.execute("SELECT * FROM error")
    error_rows = db_cursor.fetchall()

    db_conn.commit()
    db_conn.close()

    # convert lists to dataframe
    measurement_items = ['ID', 'DATETIME']
    for i in dbconfig['estimator']['measurement']:
        measurement_items.append(i[0])
    measurement_data = pd.DataFrame(measurement_rows)
    measurement_data.columns = measurement_items
    measurement_data['DATETIME'] = measurement_data['DATETIME'].astype(float)

    state_items = ['ID', 'DATETIME']
    for i in dbconfig['estimator']['state']:
        state_items.append(i[0])
    state_data = pd.DataFrame(state_rows)
    state_data.columns = state_items
    state_data['DATETIME'] = state_data['DATETIME'].astype(float)

    error_items = ['ID', 'DATETIME']
    for i in dbconfig['estimator']['error']:
        error_items.append(i[0])
    error_data = pd.DataFrame(error_rows)
    error_data.columns = error_items
    error_data['DATETIME'] = error_data['DATETIME'].astype(float)

    return measurement_data, state_data, error_data


def parse_marineradar(db_path, db_config_path, num_spokedata):
    # Read the db config file
    with open(db_config_path) as f:
        dbconfig = json.load(f)

    # connect to sqlite database
    db_conn = sqlite3.connect(db_path)
    # create a cursor to execute SQL commands
    db_cursor = db_conn.cursor()

    #  retrieve data from database
    db_cursor.execute("SELECT * FROM radar")
    radar_rows = db_cursor.fetchall()
    db_conn.commit()
    db_conn.close()

    # radar dataframe
    radar_items = ['ID', 'DATETIME']
    for i in dbconfig['marineradar']:
        radar_items.append(i[0])
    radar_data = pd.DataFrame(radar_rows)
    radar_data.columns = radar_items
    radar_data['DATETIME'] = radar_data['DATETIME'].astype(float)

    # parse the BLOB data
    unpack_formats = '=' + str(num_spokedata) + 'B'  # unsigned char
    spokedata = []
    for onerow in radar_data['SpokeData']:
        spokedata.append(struct.unpack(unpack_formats, onerow))

    radar_data['SpokeData'] = spokedata

    return radar_data
