# -*- coding: utf-8 -*-
import json
import os
import sys
import optparse
from benchmark_controller import Benchmark, nnnBenchmark
import time

print("------------ Importing environment packages --------------")

ENV_SELECTION = {}
simbenchmark_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(simbenchmark_dir)
sys.path.insert(0, os.getcwd())


def import_manager(sim_name):
    if sim_name == "webots":
        try:
            from environments.WeBots.controller import RobotEnv_webots
            from environments.WeBots.controller import nnnEnv_webots

            ENV_SELECTION.update({"webots": [RobotEnv_webots, nnnEnv_webots]})
            print("WeBots packages successfully imported\n")
        except Exception as e:
            print("{}. \nWas not able to import WeBots!\n".format(e))

    if sim_name == "pybullet":
        try:
            from environments.PyBullet.controller import RobotEnv_PyBullet
            from environments.PyBullet.controller import nnnEnv_PyBullet

            ENV_SELECTION.update(
                {"pybullet": [RobotEnv_PyBullet, nnnEnv_PyBullet]})
            print("PyBullet packages successfully imported\n")
        except Exception as e:
            print("{}. \nWas not able to import PyBullet!\n".format(e))

    if sim_name == "mujoco":
        try:
            from environments.MuJoCo.controller import RobotEnv_mujoco
            from environments.MuJoCo.controller import nnnEnv_mujoco

            ENV_SELECTION.update({"mujoco": [RobotEnv_mujoco, nnnEnv_mujoco]})
            print("MuJoCo packages successfully imported\n")
        except Exception as e:
            print("{}. \nWas not able to import MuJoCo!\n".format(e))

    if sim_name == "gazebo":
        try:
            # correct import for python2 (ROS melodic)
            if sys.version_info <= (3, 4):
                simbenchmark_dir = os.path.dirname(os.path.abspath(__file__))
                sys.path.insert(0, simbenchmark_dir +
                                "/environments/Gazebo/controller")
                from robot_env import RobotEnv_gazebo
                from nnn_env import nnnEnv_gazebo
            else:
                from environments.Gazebo.controller import RobotEnv_gazebo
                from environments.Gazebo.controller import nnnEnv_gazebo
            ENV_SELECTION.update({"gazebo": [RobotEnv_gazebo, nnnEnv_gazebo]})
            print("Gazebo packages successfully imported\n")
        except Exception as e:
            print("{}. \nWas not able to import Gazebo!\n".format(e))


if __name__ == "__main__":
    optParser = optparse.OptionParser(usage="usage: %prog  [options]")
    optParser.add_option(
        "--path",
        dest="path",
        default=None,
        help="Specifies the result folder."
    )
    optParser.add_option(
        "--sim",
        dest="sim",
        default=None,
        help="Specifies the sim_name"
    )
    optParser.add_option(
        "--sim_option",
        dest="sim_option",
        default="RobotSim",
        help="RobotSim or nnnSim"
    )
    optParser.add_option(
        "--render",
        dest="render",
        default=0,
        help="headless mode [0, 1]"
    )
    options, args = optParser.parse_args()
    path = options.path
    sim_name = options.sim
    sim_option = options.sim_option
    render = options.render
    render = int(render)
    render = bool(render)

    # --------- Activate if starting through python -------
    # path = "."
    # sim_name = 'pybullet'
    # sim_name = 'webots'
    # sim_name = 'mujoco'
    # sim_name = 'gazebo'

    # sim_option = 'nnnSim'
    # render = True

    import_manager(sim_name)

    # --------- Checking is simulation environemt is available --------
    try:
        assert sim_name is not None, "Please start benchmark with shell script!"
        assert sim_name in ENV_SELECTION.keys(), sim_name + " is not available!"
    except:
        print(" ")
        print(sim_name + " not available!\n")
        sys.exit()

    if sim_option == "RobotSim":
        sim_env = ENV_SELECTION[sim_name][0]
        # initialize benchmark with simulation specific robot controller
        benchmark = Benchmark(sim_env, render=render)
    elif sim_option == "nnnSim":
        sim_env = ENV_SELECTION[sim_name][1]
        # initialize benchmark with simulation specific robot controller
        benchmark = nnnBenchmark(sim_env, render=render)
    else:
        print("ERROR: Benchmark configuration failed!")
        sys.exit()

    # number of runs per timestep
    runs_per_timestep = 1

    # Custom list of time steps to be applied in the benchmark run
    t_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
              12, 14, 16, 18, 20, 24, 28, 32, 48, 64]

    for timestep in t_list:
        for i in range(runs_per_timestep):
            # run the benchmark and get the logged data back
            t0 = time.time()
            print(
                "\nStarting {} {} with:\ntimestep {}ms\nrun {}".format(
                    sim_name, sim_option, timestep, i
                )
            )
            try:
                obs_recording = benchmark.run_benchmark(
                    sim_name,
                    timestep,
                    os.getcwd()
                    + "/benchmark_controller/simulation_sequences/data.json",
                )
            except Exception as e:
                print(e)
            print("Duration", time.time() - t0)
            print("Saving observation to file...\n")
            # save obs_recording to file
            with open(path + "/" + sim_name + "_" + sim_option + "_obs_"
                      + str(timestep) + "ms_run_" + str(i) + ".json",
                      "w",) as outfile:
                json.dump(obs_recording, outfile)

            time.sleep(5)
        time.sleep(15)
