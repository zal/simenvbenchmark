#!/usr/bin/env python3

import optparse
import os
import pandas as pd

from hardware_data_assigner import DataAssigner

optParser = optparse.OptionParser(usage='usage: %prog  [options]')
optParser.add_option('--path', dest='path', default=None,
                     help='Specifies the result folder.')
options, args = optParser.parse_args()
path = options.path

pd.set_option('display.float_format', lambda x: '%.3f' % x)


def extract_data(data_point):
    """ Extracts all relevant data from collectl output file """

    data_point_dict = {}
    cpu_data = []

    # Time
    data_point_dict.update({'abs_time': float(data_point[0][6][1:-1])})

    # CPU usage
    for line in data_point[3:]:
        if line[0] != '#':
            cpu_data.append(
                {'cpu' + line[0] + "_user": int(line[1]),
                 'cpu' + line[0] + "_idle": int(line[10])})
        else:
            break

    data_point_dict.update({'cpu': cpu_data})

    # Disk -> Only recognizes the first disk. Second disk will be ignored!
    for i in range(len(data_point)):
        if data_point[i][1] == "KBytes":
            data_point_dict.update({'disk': {'reads_kbyte': int(
                data_point[i + 1][1]),
                'writes_kbyte': int(data_point[i + 1][6])}})

    # Memory
    for i in range(len(data_point)):
        if data_point[i][1] == "Total":
            data_point_dict.update(
                {'memory': {'memory_used': data_point[i + 1][2],
                            'swap_used': data_point[i + 1][13]}})

    return data_point_dict


def get_data(path):
    """ Load data from .raw file and return list with data"""
    data = None
    data_array = []

    for filename in os.listdir(path):
        if not filename.endswith('.raw'):
            continue
        fullpath = os.path.join(path, filename)

        with open(fullpath, 'rb') as f:
            data = f.read()
            data_array.append(data)

    for i in range(len(data_array)):
        data_array[i] = data_array[i].decode("utf-8")
        data_array[i] = data_array[i].splitlines()

    return data_array


def slice_data(data):
    data_set = []
    data_point = []

    for i in range(len(data)):
        line = data[i].split()
        try:
            if line[1] != 'RECORD':
                data_point.append(line)
            else:
                data_set.append(data_point)
                data_point = []
                data_point.append(line)
        except:
            pass

    data_set = data_set[1:]

    return data_set


print('*************** Converting result files ******************')

data_array = get_data(path)

df = None

for data in data_array:
    data_set = slice_data(data)

    data_point_collection = []

    for data_point in data_set:
        data_point_collection.append(extract_data(data_point))

    # ------------- import data to DataFrame -----------
    new_df = pd.DataFrame(data_point_collection)
    if df is None:
        df = new_df
    else:
        df = pd.concat([df, new_df])  # adds data point to DataFrame

    df = df.sort_values(by=['abs_time'])  # sort data by timestemp
    df = df.reset_index(drop=True)  # Reset index

DataAssigner(path, df)
