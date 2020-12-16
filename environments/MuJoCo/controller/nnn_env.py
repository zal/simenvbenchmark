#!/usr/bin/env python3

from .simulation_interface import MujocoEnv
import time


class nnnEnv_mujoco():
    def __init__(self, render=False):
        """initializes the simulation specific nnnEnv we parsed from the
        instance-manager

        Args:
            render (bool, optional): Whether the simulation should be
            rendered or run headless. Defaults to False.
        """

        model_path = "../environment/nnnSim.xml"
        self.simulation = MujocoEnv(model_path, 1, render)
        self.sim = self.simulation.sim
        self.data = self.sim.data
        self.model = self.sim.model
        self.timestep = self.model.opt.timestep
        self.action = []
        self.sim_time = 0.0

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()

    def execute_n_timesteps(self, n=1):
        """Performing simulation step and keeping track on simulation
        time

        Args:
            n (int, optional): Amount of simulation steps to be
            performned. Defaults to 1.
        """

        self.simulation.do_simulation(self.action, n)
        self.action = []
        self.sim_time = round(self.data.time, 4)

    def reset(self):
        self.simulation.reset()

    def set_timestep(self, timestep):
        self.sim.model.opt.timestep = timestep / 1000

    def get_obs(self):
        """get observation from simulation

        Returns:
            list: List of observations
        """
        # Time
        sys_time = time.time()
        sys_timer = sys_time - self.sys_start_time

        self.sim_time = round(self.data.time, 4)

        # ball positions
        ball_positions = []
        for id in range(216):
            state = self.data.get_body_xpos("ball_%s" % id).tolist()
            ball_positions.append(state)

        return [sys_timer, self.sim_time, ball_positions]
