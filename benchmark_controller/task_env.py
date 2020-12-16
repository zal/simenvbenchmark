#!/usr/bin/env python3

from math import floor
import json
import time
from hardware_monitoring import HardwareMonitor
import numpy as np


class Benchmark:
    def __init__(self, RobotEnv, render=False):
        """Initializes the Task Environment

        Args:
            RobotEnv (object): RobotEnv object
            render (bool, optional): Whether the simulation should be
            rendered or run headless. Defaults to False.
        """

        self.RobotEnv = RobotEnv(render=render)
        self.HM = HardwareMonitor()
        self.process_counter = None

    def step(self, n_steps, arm_pos, gripper_pos, aquire_obs=False):
        """Performing an observation step

        Args:
            n_steps (int): Amount of simulation steps to be performed
            arm_pos (list of float): List of 6 joint positions for robot arm
            gripper_pos (list of float): List of 11 joint positions for 3Finger gripper
            aquire_obs (bool, optional): Whether an observation is desired. Defaults to False.

        Returns:
            obs (list): List of observations
            timings (list): List of timestemps for performance monitoring
        """

        t0 = time.perf_counter()
        # set arm joint positions
        self.RobotEnv.set_arm_pos(arm_pos)

        # set gripper joint positions
        self.RobotEnv.set_gripper_pos(gripper_pos)

        t1 = time.perf_counter()
        # execute n_steps number of timesteps in the simulation
        self.RobotEnv.execute_n_timesteps(n_steps)

        # return the observation data after the step
        t2 = time.perf_counter()

        if aquire_obs:
            # return the observation data after the step
            obs = self.RobotEnv.get_obs()
        else:
            obs = None
        t3 = time.perf_counter()
        return obs, [t0, t1, t2, t3]

    def run_benchmark(self, simulation, timestep, control_filehandle):
        """Setup and control of benchmark run
        Caution: Gazebo is running with an separate control module

        Args:
            simulation (string): Name of the simulation environment
            timestep (int): Used time step
            control_filehandle (string): Control data

        Returns:
            [dict]: Observation data of benchmark run
        """

        # Init a new obs_recording dictionary to store all the
        # observations for this benchmark run
        self.init_obs_recording(simulation, timestep)

        # set the timestep in the simulation
        self.RobotEnv.set_timestep(timestep)
        self.timestep = timestep

        # open the control data file
        with open(control_filehandle) as json_file:
            control_data = json.load(json_file)

        n_steps = 1

        # ctrl_interval length in the command_data file
        ctrl_interval = control_data["time"][1]

        # the max sim time (last entry in our control data file)
        t_max = control_data["time"][-1]

        # When to get observation data
        t_obs = 0.0
        obs_interval = 1.0

        # reset the simulation
        self.RobotEnv.reset()

        # Reset sys_start_time
        self.RobotEnv.sys_start_time = time.perf_counter()

        # main control loop
        t = 0
        t_print = 0
        armPosArr = []
        gripPosArr = []

        for i in range(len(control_data["commands"])):
            armPosArr.append(control_data["commands"][i]["arm"])
            gripPosArr.append(control_data["commands"][i]["gripper"])

        armPosArr = np.array(armPosArr)
        gripPosArr = np.array(gripPosArr)
        timer_list = []
        t_offset = self.RobotEnv.sim_time

        if simulation == "gazebo":
            # delay in seconds of when to start the trajectory
            delay = 5
            self.RobotEnv.pause()
            # parse entire data.json trajectory to gazebo
            self.RobotEnv.set_trajectory(control_data, delay)
            self.RobotEnv.unpause()
            t_offset += delay

        # exact timer start of benchmark loop
        self.t_start_recording = time.perf_counter()

        while t < t_max:
            # gets the appropriate control-index for the current sim time
            # floor() rounds down, so we always get a valid control index
            index = int(floor(t / ctrl_interval))

            # get the arm and gripper pos for the current sim time
            t1 = control_data["time"][index]
            t2 = control_data["time"][index + 1]
            ap1 = armPosArr[index]
            ap2 = armPosArr[index + 1]
            gp1 = gripPosArr[index]
            gp2 = gripPosArr[index + 1]

            arm_pos = list(ap1 + (t - t1) * (ap2 - ap1) / (t2 - t1))
            gripper_pos = list(gp1 + (t - t1) * (gp2 - gp1) / (t2 - t1))

            # Getting observation
            if t >= t_obs:
                # execute a control step and record the returned obs data
                obs, timer = self.step(
                    1, arm_pos, gripper_pos, aquire_obs=True)
                t_obs += obs_interval
                self.record_obs(obs, t_sim=t)
            else:
                obs, timer = self.step(
                    n_steps, arm_pos, gripper_pos, aquire_obs=False)

            timer_list.append([time.time(), t, timer])

            # increment the time, for the next control command selection
            t = round(self.RobotEnv.sim_time - t_offset, 5)

            # print status update every 0.2 seconds
            if t > t_print:
                t_print = t + 0.2
                print("Sim status: %s%%  Sim time: %s s" % (
                    round(t * 100 / t_max, 1),
                    round(t, 1)),
                    end="\r",)

        self.record_timings(timer_list)

        # end of the benchmark run
        return self.obs_recording

    def record_timings(self, timer_list):
        """Calculates timer information for performance monitoring

        Args:
            timer_list (list of floats): List of timer values
        """

        timer_list = np.array(timer_list, dtype=object)
        # what resolution the exported data should have, in sim time
        t_sample = 0.1

        abs_time = list(timer_list[:, 0])
        sim_time = list(timer_list[:, 1])
        timers = timer_list[:, 2]
        t_fromStart = [0]
        dt_setPos = [0]
        dt_simStep = [0]
        dt_getObs = [0]

        for timer in timers:
            t_fromStart.append(timer[0] - self.t_start_recording)
            dt_setPos.append(dt_setPos[-1] + timer[1] - timer[0])
            dt_simStep.append(dt_simStep[-1] + timer[2] - timer[1])
            dt_getObs.append(dt_getObs[-1] + timer[3] - timer[2])

        del t_fromStart[0]
        del dt_setPos[0]
        del dt_simStep[0]
        del dt_getObs[0]

        t_axis = list(np.arange(0, sim_time[-1], t_sample))
        t_abs = list(np.interp(t_axis, sim_time, abs_time))
        t_fromStart = list(np.interp(t_axis, sim_time, t_fromStart))
        t_setPos = list(np.interp(t_axis, sim_time, dt_setPos))
        t_simStep = list(np.interp(t_axis, sim_time, dt_simStep))
        t_getObs = list(np.interp(t_axis, sim_time, dt_getObs))

        rtf_sim = []

        for i in range(len(t_simStep) - 1):
            rtf_sim.append(t_sample / (t_simStep[i + 1] - t_simStep[i]))
        rtf_sim.append(rtf_sim[-1])

        self.obs_recording["timings"] = {
            "t_sim": t_axis,  # sim time (x-axis for graphs)
            "t_abs": t_abs,  # absolute computer time
            "t_fromStart": t_fromStart,  # stopwatch of benchmark run in s
            "t_exec_setPos": t_setPos,  # cumulative time spent setting controllers
            "t_exec_simStep": t_simStep,  # cumulative time spent executing the sim
            "t_exec_getObs": t_getObs,  # cumulative time spent getting obs
            "rtf_sim": rtf_sim  # realtime factor of simulation
        }

    def init_obs_recording(self, simulation, timestep):
        """Initializes the JSON result file

        Args:
            simulation (string): Name of the simulation environment
            timestep (int): Used time step
        """

        pose_format_selection = {
            "gazebo": "quaternion",
            "pybullet": "quaternion",
            "webots": "matrix",
            "mujoco": "quaternion"
        }

        self.obs_recording = {
            "system": self.HM.get_base_info(),
            "simulation": simulation,
            "timestep": timestep,
            "start_time": self.RobotEnv.sys_start_time,
            "rotation_format": pose_format_selection[simulation],
            "keys": {
                "arm": self.RobotEnv.arm_JointNames,
                "gripper": self.RobotEnv.gripper3f_jointNames,
                "cylinders": self.RobotEnv.cylinder_names
            },
            "obs": [],
        }

    def record_obs(self, obs, t_sim):
        """ Setup data point """

        data_point = {
            "sim_time": t_sim,
            "arm": obs[2],
            "gripper": obs[3],
            "cylinders": obs[4]
        }
        self.obs_recording["obs"].append(data_point)
