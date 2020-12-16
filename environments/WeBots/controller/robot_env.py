#!/usr/bin/env python3


import time

from .simulation_interface import WebotsInterface


ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER3F_JOINT_NAMES = [
    "palm_finger_1_joint",
    "finger_1_joint_1",
    "finger_1_joint_2",
    "finger_1_joint_3",
    "palm_finger_2_joint",
    "finger_2_joint_1",
    "finger_2_joint_2",
    "finger_2_joint_3",
    "finger_middle_joint_1",
    "finger_middle_joint_2",
    "finger_middle_joint_3",
]

CYLINDER_NAMES = [
    "cylinder_1",
    "cylinder_2_1",
    "cylinder_2_2",
    "cylinder_2_3",
    "cylinder_2_4",
    "cylinder_2_5",
    "cylinder_2_6",
    "cylinder_2_7",
    "cylinder_2_8",
    "cylinder_3_1",
    "cylinder_3_2",
    "cylinder_3_3",
    "cylinder_3_4",
    "cylinder_3_5",
    "cylinder_3_6",
    "cylinder_3_7",
    "cylinder_3_8",
    "cylinder_3_9",
    "cylinder_3_10",
    "cylinder_3_11",
    "cylinder_3_12",
]

INIT_POS = {
    ARM_JOINT_NAMES[0]: {"joint_index": 0, "target": 1.411872641811951},
    ARM_JOINT_NAMES[1]: {"joint_index": 1, "target": -0.8420041879132294},
    ARM_JOINT_NAMES[2]: {"joint_index": 2, "target": 1.0565325644929087},
    ARM_JOINT_NAMES[3]: {"joint_index": 3, "target": -1.7859626190435058},
    ARM_JOINT_NAMES[4]: {"joint_index": 4, "target": -1.5704882948679586},
    ARM_JOINT_NAMES[5]: {"joint_index": 5, "target": 1.4109012658608595},
}


class RobotEnv_webots:
    def __init__(self, render=False):

        self.simulation = WebotsInterface()
        self.sim_interface = self.simulation.sim_interface
        self.timestep = int(self.sim_interface.getBasicTimeStep())

        # ------------init Gripper---------
        self.GripperPos = 0.0
        self.gripper_minPos = []
        self.gripper_maxPos = []
        self.gripper_maxTorque = []
        self.gripper_motors = []
        self.gripper_sensors = []
        self.gripper3f_jointNames = GRIPPER3F_JOINT_NAMES
        self.arm_JointNames = ARM_JOINT_NAMES
        self.cylinder_names = CYLINDER_NAMES

        for name in self.gripper3f_jointNames:
            self.gripper_motors.append(self.sim_interface.getMotor(name))
            try:
                self.gripper_minPos.append(
                    self.gripper_motors[-1].getMinPosition())
            except:
                self.gripper_minPos.append(0)
            self.gripper_maxPos.append(
                self.gripper_motors[-1].getMaxPosition())
            self.gripper_maxTorque.append(
                self.gripper_motors[-1].getMaxTorque())
            self.gripper_sensors.append(
                self.sim_interface.getPositionSensor(name + "_sensor")
            )
            self.gripper_sensors[-1].enable(self.timestep)

        gt = [100] * 11
        # middle finger
        gt[8] = 8  # joint 1
        gt[10] = 8  # joint 3
        # finger 1 and finger 2
        gt[1] = gt[5] = 4
        gt[3] = gt[7] = 8
        gv = [2] * 11
        for i in range(len(self.gripper_motors)):
            self.gripper_motors[i].setAvailableTorque(gt[i])
            self.gripper_motors[i].setVelocity(gv[i])
        # ------------init Robot---------
        self.motors = []
        self.arm_sensors = []
        for name in self.arm_JointNames:
            self.motors.append(self.sim_interface.getMotor(name))
            self.arm_sensors.append(
                self.sim_interface.getPositionSensor(name + "_sensor")
            )
            self.arm_sensors[-1].enable(self.timestep)

        # ------------init cylinders---------
        self.cylinder_nodes = []
        for name in self.cylinder_names:
            self.cylinder_nodes.append(self.sim_interface.getFromDef(name))

        # ----------- Start Timer -----------
        self.sys_start_time = time.perf_counter()
        self.sim_time = 0.0

    def execute_n_timesteps(self, n=1):
        """ Performing simulation step and keeping track on simulation time

        Args:
            n (int, optional): Amount of simulation steps to be performed. Defaults to 1.
        """

        self.sim_interface.step(n * self.timestep)
        self.sim_time = self.sim_interface.getTime()

    def set_arm_pos(self, positions):
        """ Set target joint positions of robot arm

        Args:
            pos_percent (list): List of 6 joint positions
        """

        for i in range(len(positions)):
            self.motors[i].setPosition(positions[i])

    def set_gripper_pos(self, pos_percent):
        """ Set target joint positions of 3fGripper

        Args:
            pos_percent (list): List of 11 joint positions between 0 and 1
        """
        # make sure, pos_percent and torque_percent are values between 0 and 1
        # pos_percent = min(1, max(0, pos_percent))

        # convert percentage to actual position based on min and max position of joint
        for i in range(len(pos_percent)):
            self.gripper_motors[i].setPosition(
                self.gripper_minPos[i]
                + pos_percent[i] *
                (self.gripper_maxPos[i] - self.gripper_minPos[i])
            )

    def reset(self):
        """ Reset simulation """

        self.simulation.resetSim()

        # Get observation
        obs = self.get_obs()
        self.simulation.execute_n_timesteps(1)
        self.simulation.execute_n_timesteps(1)

        # Reset simulation and sim time
        self.simulation.resetSim()
        self.sim_time = 0.0
        self.sys_timer_start = time.perf_counter()

        return obs

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

        # Arm joints
        arm_positions = []
        for sensor in self.arm_sensors:
            arm_positions.append(sensor.getValue())

        # Gripper joints
        gripper_positions = []
        for sensor in self.gripper_sensors:
            gripper_positions.append(sensor.getValue())

        # Cylinder positions
        cylinder_states = []
        for cylinder in self.cylinder_nodes:
            pos = cylinder.getPosition()
            rot = cylinder.getOrientation()

            # state = position [3] and orientation as quaternion [4]
            state = [pos, rot]
            cylinder_states.append(state)

        return [
            sys_timer,
            self.sim_time,
            arm_positions,
            gripper_positions,
            cylinder_states,
        ]
