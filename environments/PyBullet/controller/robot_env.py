#!/usr/bin/env python3


import time
import os


from .simulation_interface import PyBulletInterface


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


class RobotEnv_PyBullet:
    def __init__(self, render=False):
        self.isRender = render
        self.simulation = PyBulletInterface(self.isRender)
        self.scene = None
        self.time_step = None

        # Camera setup
        if render:
            self.simulation._p.resetDebugVisualizerCamera(
                cameraDistance=1.3,
                cameraYaw=112,
                cameraPitch=-18,
                cameraTargetPosition=[0, 0.7, 0.3])

        # ------------ setup simulation environment ---------
        self.model_path = (
            os.getcwd() + r"/environments/PyBullet/environment/test_case_main_MJCF.xml"
        )
        self.robot_path = (
            os.getcwd() + r"/environments/PyBullet/environment/urdfs/ur10e.urdf"
        )

        self.gripper3f_jointNames = GRIPPER3F_JOINT_NAMES
        self.arm_JointNames = ARM_JOINT_NAMES
        self.cylinder_names = CYLINDER_NAMES
        self.init_pos = INIT_POS

        self.objects, self.robot = self.simulation.load_model(
            self.model_path, self.robot_path
        )
        self.simulation.reset_joint_state(self.robot, self.init_pos)

        robot_info = self.simulation.get_joint_info(self.robot)

        # ------------init Robot---------
        self.robot_joints = {}

        for name in self.arm_JointNames:
            robot_joint = {
                name: {
                    "joint_index": None,
                    "max_force": None,
                    "max_vel": None,
                    "min_pos": None,
                    "max_pos": None,
                    "target": None,
                    "vel_target": None,
                }
            }
            for e in robot_info:
                if str(e[1].decode("UTF-8")) == name:
                    robot_joint[name]["joint_index"] = e[0]
                    robot_joint[name]["max_force"] = e[10]
                    robot_joint[name]["min_pos"] = e[8]
                    robot_joint[name]["max_pos"] = e[9]
                    robot_joint[name]["max_vel"] = e[11]

            self.robot_joints.update(robot_joint)

        # ------------ init Gripper -----------
        self.gripper_joints = {}

        for name in self.gripper3f_jointNames:
            gripper_joint = {
                name: {
                    "joint_index": None,
                    "max_force": None,
                    "max_vel": None,
                    "min_pos": None,
                    "max_pos": None,
                    "target": None,
                }
            }
            for e in robot_info:
                if str(e[1].decode("UTF-8")) == name:
                    gripper_joint[name]["joint_index"] = e[0]
                    gripper_joint[name]["max_force"] = e[10]
                    gripper_joint[name]["min_pos"] = e[8]
                    gripper_joint[name]["max_pos"] = e[9]
                    gripper_joint[name]["max_vel"] = e[11]

            self.gripper_joints.update(gripper_joint)

        # ------------init cylinders---------
        self.cylinder_nodes = []
        for name in self.cylinder_names:
            pass

        # ----------- Simulation Time -----------
        self.sim_time = 0

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()

    def execute_n_timesteps(self, n=1):
        """ Performing simulation step and keeping track on simulation time

        Args:
            n (int, optional): Amount of simulation steps to be performned. Defaults to 1.
        """

        self.sim_time += n * (self.time_step / 1000.0)
        self.simulation.do_simulation(n)

    def calc_velocity(self, act_pos, target_pos):
        """ Calculating target joint velocities

        Args:
            act_pos (list): Actual joint positions
            target_pos (list): Target joint positions

        Returns:
            [type]: [description]
        """

        P = 10.0
        D = 0.0
        I = 0.0

        error = target_pos - act_pos
        vel = P * error

        return vel

    def set_arm_pos(self, positions):
        """ Set target joint positions of robot arm

        Args:
            pos_percent (list): List of 6 joint positions
        """

        # get the robot_state, this includes EVERY joint. Thus we only have to do it once and can skip for gripper
        self.robot_state = self.simulation.get_joint_state(self.robot)

        for i in range(len(positions)):
            # Target position
            self.robot_joints[self.arm_JointNames[i]]["target"] = positions[i]

            # Target velocity
            vel = self.calc_velocity(self.robot_state[i][0], positions[i])
            self.robot_joints[self.arm_JointNames[i]]["vel_target"] = vel

        # we wait for the set_gripper_pos to be called and send a combined command to pybullet
        # self.simulation.set_joint_states_vel(self.robot, self.robot_joints)

    # pos_percent is value between 0 and 1
    def set_gripper_pos(self, pos_percent):
        """ Set target joint positions of 3fGripper

        Args:
            pos_percent (list): List of 11 joint positions between 0 and 1
        """
        # make sure, pos_percent are values between 0 and 1
        # pos_percent = min(1, max(0, pos_percent))
        # convert percentage to actual position based on min and max position of joint

        # next line is uncommented, because we are are calling set_arm_pos first
        # self.robot_state = self.simulation.get_joint_state(self.robot)
        for i in range(len(pos_percent)):
            # Target position
            min_pos = self.gripper_joints[self.gripper3f_jointNames[i]]["min_pos"]
            max_pos = self.gripper_joints[self.gripper3f_jointNames[i]]["max_pos"]
            target = min_pos + pos_percent[i] * (max_pos - min_pos)

            self.gripper_joints[self.gripper3f_jointNames[i]
                                ]["target"] = target

            # Target velocity
            index = self.gripper_joints[self.gripper3f_jointNames[i]
                                        ]["joint_index"]
            vel = self.calc_velocity(self.robot_state[index][0], target)
            self.gripper_joints[self.gripper3f_jointNames[i]
                                ]["vel_target"] = vel

        # We send one request to set_joint_states_vel with the merged dictionaries of gripper and arm
        self.simulation.set_joint_states_vel(
            self.robot, {**self.gripper_joints, **self.robot_joints}
        )

    def reset(self):
        """ Reset simulation """

        self.sys_timer_start = time.perf_counter()
        self.objects, self.robot = self.simulation.reset(
            self.model_path, self.robot_path, self.init_pos
        )
        self.simulation.set_physic_parameters()
        self.sim_time = 0

    def set_timestep(self, timestep):
        """ Define simulation timestep

        Args:
            timestep (int): Simulation timestep value
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

        robot_state = list(self.simulation.get_joint_state(self.robot))

        # Remove fixed joints
        robot_state.pop(16)
        robot_state.pop(7)
        robot_state.pop(6)

        # Arm joints
        robot_joint_positions = []
        for i in range(len(self.arm_JointNames)):
            robot_joint_positions.append(robot_state[i])

        # Gripper joints
        gripper_joint_positions = []
        for i in range(len(self.gripper3f_jointNames)):
            gripper_joint_positions.append(robot_state[i])

        # Cylinder positions
        # The orientation is a quaternion in [x,y,z,w] format.
        cylinder_states = self.simulation.get_body_state(self.objects)[6:]

        return [
            sys_timer,
            sim_time,
            robot_joint_positions,
            gripper_joint_positions,
            cylinder_states,
        ]


if __name__ == "__main__":
    env = RobotEnv_PyBullet(True)
