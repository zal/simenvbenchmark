#!/usr/bin/env python3
from .simulation_interface import MujocoEnv
import time
import json
import numpy as np
import os


ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

GRIPPER3F_JOINT_NAMES = [
    'palm_finger_1_joint',
    'finger_1_joint_1',
    'finger_1_joint_2',
    'finger_1_joint_3',
    'palm_finger_2_joint',
    'finger_2_joint_1',
    'finger_2_joint_2',
    'finger_2_joint_3',
    'finger_middle_joint_1',
    'finger_middle_joint_2',
    'finger_middle_joint_3']

CYLINDER_NAMES = [
    'cylinder_1',
    'cylinder_2_1',
    'cylinder_2_2',
    'cylinder_2_3',
    'cylinder_2_4',
    'cylinder_2_5',
    'cylinder_2_6',
    'cylinder_2_7',
    'cylinder_2_8',
    'cylinder_3_1',
    'cylinder_3_2',
    'cylinder_3_3',
    'cylinder_3_4',
    'cylinder_3_5',
    'cylinder_3_6',
    'cylinder_3_7',
    'cylinder_3_8',
    'cylinder_3_9',
    'cylinder_3_10',
    'cylinder_3_11',
    'cylinder_3_12',
]


class RobotEnv_mujoco():
    def __init__(self, render=False):
        """initializes the simulation specific RobotEnv we parsed from
        the instance-manager

        Args:
            render (bool, optional): Whether the simulation should be
            rendered or run headless. Defaults to False.
        """

        model_path = "../environment/RobotSim.xml"
        self.simulation = MujocoEnv(model_path, 1, render=render)
        self.sim = self.simulation.sim
        self.data = self.sim.data
        self.model = self.sim.model
        self.timestep = self.model.opt.timestep

        self.gripper3f_jointNames = GRIPPER3F_JOINT_NAMES
        self.arm_JointNames = ARM_JOINT_NAMES
        self.cylinder_names = CYLINDER_NAMES

        self.robot_state = []

        # ------------init Gripper---------
        self.action = []
        self.gripper_minPos = []
        self.gripper_maxPos = []

        for e in self.model.actuator_ctrlrange[-len(self.gripper3f_jointNames):]:
            self.gripper_minPos.append(e[0])
            self.gripper_maxPos.append(e[1])

        # ------------init Arm---------
        self.setup_init_state()

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()
        self.sim_time = 0.0

    def setup_init_state(self):
        """Setup joint position init state of simulation"""

        init_state_json = os.path.join(os.path.dirname(
            __file__), "../environment/init_state_setup.json")

        # open json file with init qpos and qvel
        init_state = {}
        with open(init_state_json) as json_file:
            init_state = json.load(json_file)

        init_state['qpos'] = np.array(init_state['qpos'])
        init_state['qvel'] = np.array(init_state['qvel'])

        self.simulation.set_state(init_state['qpos'], init_state['qvel'])

    def execute_n_timesteps(self, n=1):
        """Performing simulation step and keeping track on simulation time

        Args:
            n (int, optional): Amount of simulation steps to be
            performned. Defaults to 1.
        """
        assert len(self.action) == 17

        self.simulation.do_simulation(self.action, n)
        self.action = []
        self.sim_time = round(self.data.time, 4)

    def set_arm_pos(self, positions):
        """Set target joint positions of robot arm

        Args:
            positions (list): List of 6 joint positions
        """

        if positions == None:
            positions = self.sim.data.ctrl[:len(self.arm_jointNames)]

        for i in positions:
            self.action.append(i)

    def set_gripper_pos(self, pos_percent):
        """Set target joint positions of 3fGripper

        Args:
            pos_percent (list): List of 11 joint positions between 0 and 1
        """
        # make sure, pos_percent and torque_percent are values between 0 and 1
        # convert percentage to actual position based on min and max position of joint

        if pos_percent == None:
            pos = self.sim.data.ctrl[-len(self.gripper3f_jointNames):]
            self.set_gripper_pos_explicit(pos)
        else:
            for i in range(len(pos_percent)):
                self.action.append(
                    self.gripper_minPos[i]
                    + pos_percent[i] * (self.gripper_maxPos[i]-self.gripper_minPos[i]))

    def reset(self):
        """Reset simulation"""

        self.simulation.reset()
        self.setup_init_state()
        self.sim_time = 0.0

    def set_timestep(self, timestep):
        """Define simulation timestep

        Args:
            timestep (int): Simulation timestep value
        """

        self.sim.model.opt.timestep = timestep / 1000

    def get_obs(self):
        """get observation from simulation

        Returns:
            list: Timer and observation data
        """
        # Attention: Take care of actuator order

        # Time
        sys_time = time.time()
        sys_timer = sys_time - self.sys_start_time

        self.sim_time = round(self.data.time, 4)

        # Arm joints
        arm_positions = []
        for act_pos in self.data.actuator_length[:len(self.arm_JointNames)]:
            arm_positions.append(act_pos)

        # Gripper joints
        gripper_positions = []
        for act_pos in self.data.actuator_length[
                len(self.arm_JointNames):len(self.arm_JointNames)
                + len(self.gripper3f_jointNames)]:
            gripper_positions.append(act_pos)

        # Cylinder positions
        cylinder_positions = []

        for name in self.cylinder_names:
            pos = self.data.get_body_xpos(name).tolist()
            rot = self.data.get_body_xquat(name).tolist()
            state = [pos, rot]
            cylinder_positions.append(state)

        return [sys_timer, self.sim_time, arm_positions,
                gripper_positions, cylinder_positions]
