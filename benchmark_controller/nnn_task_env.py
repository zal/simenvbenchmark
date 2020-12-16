#!/usr/bin/env python3

import time
from hardware_monitoring import HardwareMonitor
import numpy as np


class nnnBenchmark:
    def __init__(self, nnnEnv, render=False):
        """Initializes the Task Environment

        Args:
            nnnEnv (object): nnnEnv object
            render (bool, optional): Whether the simulation should be
            rendered or run headless. Defaults to False.
        """

        self.nnnEnv = nnnEnv(render=render)
        self.HM = HardwareMonitor()
        self.process_counter = None

    def step(self, aquire_obs=False):
        """Performing an observation step

        Args:
            aquire_obs (bool, optional): Whether an observation is
            desired. Defaults to False.

        Returns:
            obs (list): List of observations
            timings (list): List of timestemps for performance
            monitoring
        """

        t0 = time.perf_counter()

        t1 = time.perf_counter()
        # execute n_steps number of timesteps in the simulation
        self.nnnEnv.execute_n_timesteps()

        t2 = time.perf_counter()

        obs = None
        if aquire_obs is True:
            # return the observation data after the step
            obs = self.nnnEnv.get_obs()

        t3 = time.perf_counter()

        return obs, [t0, t1, t2, t3]

    def run_benchmark(self, simulation, timestep, control_filehandle):
        """Setup and control of benchmark run
        Caution: Gazebo is running with an separate control module

        Args:
            simulation (string): Name of the simulation environment
            timestep (int): Used time step
            control_filehandle (string): not used in nnnSim

        Returns:
            [dict]: Observation data of benchmark run
        """

        # Init a new obs_recording dictionary to store all the
        # observations for this benchmark run
        self.init_obs_recording(simulation, timestep)

        # set the timestep in the simulation
        self.nnnEnv.set_timestep(timestep)

        # the max sim time (last entry in our control data file)
        t_max = 4.0
        t = 0.0

        # reset the simulation
        self.nnnEnv.reset()

        # When to get observation data
        t_obs = 0.0
        obs_interval = 0.02

        # Reset sys_start_time
        self.nnnEnv.sys_start_time = time.time()
        # self.nnnEnv.sim_time = 0.0

        # main control loop
        t = 0
        t_print = 0
        timer_list = []
        self.t_start_recording = time.perf_counter()

        # seperate loop for Gazebo
        if simulation == "gazebo":
            # run simulation for t_max seconds
            self.nnnEnv.unpause()
            while t < t_max:
                # record timings info
                obs, timer = self.step(aquire_obs=False)
                timer_list.append([time.time(), t, timer])
                t = round(self.nnnEnv.sim_time, 5)
                if t > t_print:
                    t_print = t + 0.2
                    print(
                        "Sim status: %s%%  Sim time: %s s"
                        % (round(self.nnnEnv.sim_time * 100 / t_max, 1),
                           round(t, 1)),
                        end="\r",
                    )
            self.nnnEnv.pause()

            # gazebo nnn_env.py recorded every model_states message
            # from ROS in the next code block, the ball positions get
            # extracted
            t_sim_list = []
            balls_list = []
            for t_sim, model_states in self.nnnEnv.model_states_list:
                t_sim_list.append(t_sim)
                balls = []
                for pose in model_states.pose:
                    pos = pose.position
                    balls.append([pos.x, pos.y, pos.z])
                del balls[0]
                balls_list.append(np.array(balls).flatten())

            # t_axis are the sim timesteps we want to get ball positions
            t_axis = list(np.arange(0, 4, obs_interval))
            # this list contains every ball position 216 * 3 = 648
            # values per ROS msg
            balls_list = np.array(balls_list)

            # balls_interp is each of the 648 values interpolated
            # between 0 and 4 seconds
            balls_interp = []

            for i in range(len(balls_list[0])):
                bp_i = balls_list[:, i]
                balls_interp.append(list(np.interp(t_axis, t_sim_list, bp_i)))
            balls_interp = np.array(balls_interp)

            # here we generate the "obs" entries for the .json files
            for i in range(len(t_axis)):
                balls = []
                for j in range(216):
                    x = balls_interp[3 * j, i]
                    y = balls_interp[3 * j + 1, i]
                    z = balls_interp[3 * j + 2, i]
                    balls.append([x, y, z])
                data_point = {
                    "sim_time": t_axis[i],
                    "balls": balls,
                }
                self.obs_recording["obs"].append(data_point)
        else:
            # Init observation
            obs, timer = self.step(aquire_obs=True)

            while t < t_max:
                obs = None
                # Getting observation
                if t >= t_obs:
                    # execute a control step and record the returned obs data
                    obs, timer = self.step(aquire_obs=True)
                    t_obs += obs_interval
                    self.record_obs(obs, t_sim=t)
                else:
                    obs, timer = self.step(aquire_obs=False)

                timer_list.append([time.time(), t, timer])

                # increment the time, for the next control command selection
                t = round(self.nnnEnv.sim_time, 5)

                # print status update every 0.2 sim-seconds
                if t > t_print:
                    t_print = t + 0.2
                    print("Sim status: %s%%  Sim time: %s s" % (
                        round(self.nnnEnv.sim_time * 100 / t_max, 1),
                        round(t, 1)),
                        end="\r",)

        self.record_timings(timer_list)
        # Print line break for next print to work
        print("\n")
        # return the accumulated obs_recording after the end of
        # the benchmark run
        return self.obs_recording

    def record_timings(self, timer_list):
        """Calculates timer information for performance monitoring

        Args:
            timer_list (list of floats): List of timer values
        """

        timer_list = np.array(timer_list, dtype=object)
        # what resolution the exported data should have, in sim time
        t_sample = 0.01

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
            "mujoco": "quaternion",
        }

        self.obs_recording = {
            "system": self.HM.get_base_info(),
            "simulation": simulation,
            "timestep": timestep,
            "start_time": self.nnnEnv.sys_start_time,
            "rotation_format": pose_format_selection[simulation],
            "obs": [],
        }

    def record_obs(self, obs, t_sim):
        """ Setup data point """

        data_point = {
            "sim_time": t_sim,
            "balls": obs[2],
        }
        self.obs_recording["obs"].append(data_point)
