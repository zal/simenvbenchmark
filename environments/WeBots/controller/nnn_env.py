#!/usr/bin/env python3

# ------ Base imports ------
import time

# ------ WeBots packages --------
from .simulation_interface import WebotsInterface


class nnnEnv_webots:
    def __init__(self, render=False):

        self.simulation = WebotsInterface()
        self.sim_interface = self.simulation.sim_interface
        self.timestep = int(self.sim_interface.getBasicTimeStep())
        self.sim_time = 0.0

        # ------------init balls---------
        self.ball_nodes = []
        for i in range(216):
            self.ball_nodes.append(
                self.sim_interface.getFromDef("ball_%s" % i))

        # ----------- Start Timer -----------
        self.sys_start_time = time.perf_counter()

    def execute_n_timesteps(self, n=1):
        """ Performing simulation step and keeping track on
        simulation time

        Args:
            n (int, optional): Amount of simulation steps to be
            performned. Defaults to 1.
        """

        self.sim_interface.step(n * self.timestep)
        self.sim_time = self.sim_interface.getTime()
        # self.simulation.execute_n_timesteps(n)

    def reset(self):
        """ Reset simulation """

        self.simulation.resetSim()
        self.simulation.execute_n_timesteps(1)
        self.simulation.execute_n_timesteps(1)
        self.simulation.resetSim()
        self.sim_time = 0
        self.sys_timer_start = time.perf_counter()

    def set_timestep(self, timestep):
        """ Define simulation timestep

        Args:
            timestep (int): Simulation timestep value
        """

        self.simulation.WorldInfo.getField(
            "basicTimeStep").setSFFloat(timestep)
        self.simulation.timestep = timestep

    def get_obs(self):
        """ Get observation from simulation

        Returns:
            list: Timer and observation data
        """

        # Time
        sys_time = time.perf_counter()
        sys_timer = sys_time - self.sys_start_time

        self.sim_time = self.sim_interface.getTime()

        # ball positions
        ball_states = []
        for ball in self.ball_nodes:
            state = ball.getPosition()
            ball_states.append(state)

        return [sys_timer, self.sim_time, ball_states]
