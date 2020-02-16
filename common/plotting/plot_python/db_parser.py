# /*
# ****************************************************************************
# * db_parser.py:
# * Illustration of results of state estimator, using the estimator.db
# *
# * by Hu.ZH(CrossOcean.ai)
# ****************************************************************************
# */


import sqlite3
import json
import pandas as pd


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
