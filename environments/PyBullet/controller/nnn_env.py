#!/usr/bin/env python3


import time
import os

from .simulation_interface import PyBulletInterface


class nnnEnv_PyBullet:
    def __init__(self, render=True):
        self.isRender = render
        self.simulation = PyBulletInterface(self.isRender)
        self.scene = None
        self.time_step = None
        self.objects = None

        # ------------ setup simulation environment ---------
        self.model_path = os.getcwd() + r"/environments/PyBullet/environment/nnnSim.xml"

        self.objects, self.robot = self.simulation.load_model(self.model_path)
        self.simulation.set_physic_parameters(only_main_setting=True)

        # ----------- Simulation Time -----------
        self.sim_time = 0.0

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()

    def execute_n_timesteps(self, n=1):
        """ Performing simulation step and keeping track on simulation time

        Args:
            n (int, optional): Amount of simulation steps to be performned. Defaults to 1.
        """

        self.sim_time += n * (self.time_step / 1000.0)
        self.simulation.do_simulation(n)

    def reset(self):
        """ Reset simulation """

        self.sys_timer_start = time.perf_counter()
        self.objects, self.robot = self.simulation.reset(self.model_path)
        self.simulation.set_physic_parameters(only_main_setting=True)
        self.sim_time = 0

    def set_timestep(self, timestep):
        """ Define simulation timestep

        Args:
            timestep (int): Simulation timestep value
            ctrl_step (int): Amount of steps to perform for one command
        """

        self.simulation.set_timestep(timestep)
        self.time_step = timestep

    def get_obs(self):
        """ Get observation from simulation

        Returns:
            list: Timer and observation data
        """

        # Time
        sys_time = time.time()
        sys_timer = sys_time - self.sys_start_time

        sim_time = self.sim_time

        # ball positions
        # The orientation is a quaternion in [x,y,z,w] format.
        ball_states = self.simulation.get_body_positions(self.objects)[1:]

        return [sys_timer, sim_time, ball_states]
