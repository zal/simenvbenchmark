#!/usr/bin/env python3

import os
import json
from scipy import interpolate


class DataAssigner:
    def __init__(self, path, df):
        self.path = path
        self.df = df

        self.obs_files = self.get_all_files("json")

        self.assigne_data()

    def get_all_files(self, file_type):
        """ collect all JSON result files """

        files = []

        for filename in os.listdir(self.path):
            if not filename.endswith(file_type):
                continue
            fullpath = os.path.join(self.path, filename)
            files.append(fullpath)

        return files

    def check_if_hardware_data_exist(self, obs):
        """ checking whether JOSN data point contains hardware data """

        data_exist = False

        try:
            if len(obs['hardware_monitoring']['abs_time']) > 0:
                data_exist = True
        except:
            pass

        return data_exist

    def assigne_data(self):
        """ Assigne hardware data to JSON data point """

        counter = 0

        for obs_file in self.obs_files:
            f = open(obs_file, "r")
            obs = json.load(f)

            if self.check_if_hardware_data_exist(obs):
                continue

            timings_data = obs["timings"]

            try:
                obs_start_time = timings_data["t_abs"][0]
                obs_end_time = timings_data["t_abs"][-1]
                t_sim = timings_data["t_sim"]
                t_abs = timings_data["t_abs"]
            except:
                print("No observation found in %s" % obs_file)
                counter += 1
                continue

            # Get time specific data range
            hardware_df = self.df.loc[self.df["abs_time"] > obs_start_time]
            hardware_df = hardware_df.loc[hardware_df["abs_time"]
                                          < obs_end_time]
            hardware_dict = hardware_df.to_dict()

            if len(hardware_dict['abs_time']) > 0:
                continue

            print(" ")
            print(obs_file)

            # add interpolated sim_time values to hardware monitoring
            t_abs2sim = interpolate.interp1d(t_abs, t_sim)
            hardware_dict["t_sim_interpolated"] = hardware_dict["abs_time"]

            for k, v in hardware_dict["abs_time"].items():
                hardware_dict["t_sim_interpolated"][k] = float(t_abs2sim(v))

            # add the hardware_monitoring to our obs dict
            obs["hardware_monitoring"] = hardware_dict

            # Write JSON file
            with open(obs_file, "w") as outfile:
                json.dump(obs, outfile)

            counter += 1
            print(
                "%s of %s result files written." % (
                    counter, len(self.obs_files)),
                end="\r",
            )

            print("")
            print(len(obs['hardware_monitoring']['abs_time']))
