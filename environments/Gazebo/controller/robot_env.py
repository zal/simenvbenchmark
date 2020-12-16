#!/usr/bin/env python3

import rospy
import time
import xml.etree.ElementTree as ET

from controller_manager_msgs.srv import (
    SwitchController,
    SwitchControllerRequest
)
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ODEPhysics, ModelStates
from gazebo_msgs.srv import (
    SetPhysicsProperties,
    SetPhysicsPropertiesRequest,
    SetModelConfiguration,
    SetModelConfigurationRequest,
)
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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


class RobotEnv_gazebo:
    def __init__(self, render=False):
        # ROS Subscribers
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_callback
        )
        self.arm_states_sub = rospy.Subscriber(
            "/eff_joint_traj_controller/state",
            JointTrajectoryControllerState,
            self.arm_states_callback,
        )
        self.gripper_states_sub = rospy.Subscriber(
            "/robotiq_3f_controller/state",
            JointTrajectoryControllerState,
            self.gripper_states_callback,
        )
        self.clock = rospy.Subscriber("/clock", Clock, self.clock_callback)

        # ROS Publishers
        self.pub_arm = rospy.Publisher(
            "/ur_joint_group_pos_controller/command2", Float64MultiArray, queue_size=1
        )
        self.pub_arm_traj = rospy.Publisher(
            "/eff_joint_traj_controller/command", JointTrajectory, queue_size=1
        )
        # self.pub_gripper = rospy.Publisher(
        #     "/robotiq_joint_state_controller/command2", Float64MultiArray, queue_size=1
        # )
        self.pub_gripper_traj = rospy.Publisher(
            "/robotiq_3f_controller/command", JointTrajectory, queue_size=1
        )

        # ROS Services
        self.unpause_proxy = rospy.ServiceProxy(
            "/gazebo/unpause_physics", Empty)
        self.pause_proxy = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_sim_proxy = rospy.ServiceProxy(
            "/gazebo/reset_simulation", Empty)
        self.reset_env_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.set_physics = rospy.ServiceProxy(
            "/gazebo/set_physics_properties", SetPhysicsProperties
        )
        self.set_modelconfig = rospy.ServiceProxy(
            "/gazebo/set_model_configuration", SetModelConfiguration
        )
        self.switch_service = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )

        # ROS controllers
        self.controllers_list = [
            "ur_joint_state_controller",
            "eff_joint_traj_controller",
            "robotiq_joint_state_controller",
            "robotiq_3f_controller",
        ]
        # Physic parameters
        self._time_step = 0.001
        self._max_update_rate = 0.0

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 100
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20

        self._gravity = Vector3()
        self._gravity.x = 0.0
        self._gravity.y = 0.0
        self._gravity.z = -1.0

        # Joints
        self.joint_states = JointState()
        self.pos_cmd = Float64MultiArray()
        self.arm_JointNames = ARM_JOINT_NAMES
        self.cylinder_names = CYLINDER_NAMES
        self.gripper3f_jointNames = GRIPPER3F_JOINT_NAMES
        self.init_pos = INIT_POS

        self.arm_states = None
        self.gripper_states = None
        self.model_states = None

        # Choose controller type
        self.controller_type = "trajectory"
        # controller_type = "position"

        # Get joint limits
        self.joint_info = {}
        urdf = rospy.get_param("/robot_description")
        root = ET.XML(urdf)

        for type_tag in root.findall("joint"):
            name = type_tag.attrib["name"]

            if name in self.gripper3f_jointNames:
                limit_tag = type_tag.find("limit")
                param = {
                    "effort": limit_tag.attrib["effort"],
                    "lower": limit_tag.attrib["lower"],
                    "upper": limit_tag.attrib["upper"],
                    "velocity": limit_tag.attrib["velocity"],
                }

                self.joint_info.update({name: param})

        self.sim_time = 0.0
        self.init_sim_time = 0.0
        self.clock = 0.0

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()
        rospy.init_node("RobotEnv_gazebo", anonymous=True,
                        log_level=rospy.WARN)
        self.reset()
        self.t1 = 0
        self.rate = 1000

    def execute_n_timesteps(self, n=1):
        self.rate.sleep()

    def set_arm_pos(self, positions):
        return

    def set_gripper_pos(self, positions):
        return

    def reset(self):
        """ Reset simulation and controllers.
            Controllers are reset twice to prevent unstable behaviors.
        """
        self.unpause()
        self.reset_controllers()

        self.pause()
        self.reset_sim()
        self.set_pose()

        self.unpause()
        self.reset_controllers()
        self.check_subscribers()

        self.pause()

    def set_timestep(self, timestep):
        """ Define simulation timestep. Reset physic parameters after simulation reset.

        Args:
            timestep (int): Simulation timestep value
        """

        self._time_step = float(timestep) / 1000.0
        self.rate = rospy.Rate(1000 / timestep)
        self.change_physics()

    def get_obs(self):
        """ Get observation from simulation

        Returns:
            list: Timer and observation data
        """

        # Time
        sys_time = time.time()
        sys_timer = sys_time - self.sys_start_time

        # Arm joints
        for i in range(len(self.arm_positions)):
            self.arm_positions[i] = self.arm_states.actual.positions[
                self.arm_indices[i]
            ]

        # Gripper joints
        for i in range(len(self.gripper_positions)):
            self.gripper_positions[i] = self.gripper_states.actual.positions[
                self.gripper_indices[i]
            ]

        # Cylinder positions
        for i in range(len(self.cylinder_states)):
            cyl_pose = self.model_states.pose[self.model_indices[i]]
            pos = [cyl_pose.position.x, cyl_pose.position.y, cyl_pose.position.z]
            rot = [
                cyl_pose.orientation.w,
                cyl_pose.orientation.x,
                cyl_pose.orientation.y,
                cyl_pose.orientation.z,
            ]
            self.cylinder_states[i] = [pos, rot]

        return [
            sys_timer,
            self.sim_time,
            self.arm_positions,
            self.gripper_positions,
            self.cylinder_states,
        ]

    def set_trajectory(self, control_data, delay):
        """ Define trajectory.

        Args:
            control_data (list of float): List of joint target positions
            delay (float): Duration
        """
        arm_traj = JointTrajectory()
        arm_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(delay)
        arm_traj.joint_names = self.arm_JointNames
        gripper_traj = JointTrajectory()
        gripper_traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(delay)
        gripper_traj.joint_names = self.gripper3f_jointNames

        for i in range(len(control_data["commands"])):
            point = JointTrajectoryPoint()

            point.time_from_start = rospy.Duration.from_sec(
                control_data["time"][i])

            point.positions = control_data["commands"][i]["arm"]
            arm_traj.points.append(point)
            point2 = JointTrajectoryPoint()

            point2.time_from_start = rospy.Duration.from_sec(
                control_data["time"][i])

            point2.positions = self.get_pos_from_percent(
                control_data["commands"][i]["gripper"])

            gripper_traj.points.append(point2)

        # Wait until connection is established
        while self.pub_arm_traj.get_num_connections() == 0:
            pass
        self.pub_arm_traj.publish(arm_traj)

        # Wait until connection is established
        while self.pub_gripper_traj.get_num_connections() == 0:
            pass
        self.pub_gripper_traj.publish(gripper_traj)

    def get_pos_from_percent(self, pos_percent):
        """ Calculate joint angle position from normalized angles

        Args:
            pos_percent (list of float): normalized angles with values between 0 and 1

        Returns:
            positions (list of float): joint angle position as [rad]
        """

        positions = []
        for i in range(len(pos_percent)):
            max_pos = float(
                self.joint_info[self.gripper3f_jointNames[i]]["upper"])
            min_pos = float(
                self.joint_info[self.gripper3f_jointNames[i]]["lower"])
            target = min_pos + pos_percent[i] * (max_pos - min_pos)
            positions.append(target)

        return positions

    def arm_states_callback(self, msg):
        self.arm_states = msg

    def gripper_states_callback(self, msg):
        self.gripper_states = msg

    def model_states_callback(self, msg):
        self.model_states = msg

    def clock_callback(self, msg):
        self.sim_time = msg.clock.secs + msg.clock.nsecs * 1e-9

    def check_subscribers(self):
        """
        This services makes sure subscribers are running and create name-index lists
        """

        self.arm_positions = [0] * len(ARM_JOINT_NAMES)
        self.gripper_positions = [0] * len(GRIPPER3F_JOINT_NAMES)
        self.cylinder_states = [0] * len(CYLINDER_NAMES)
        self.arm_indices = []
        self.gripper_indices = []
        self.model_indices = []
        while not rospy.is_shutdown():
            try:
                for name in self.arm_states.joint_names:
                    self.arm_indices.append(ARM_JOINT_NAMES.index(name))
                for name in self.gripper_states.joint_names:
                    self.gripper_indices.append(
                        GRIPPER3F_JOINT_NAMES.index(name))
                for name in CYLINDER_NAMES:
                    try:
                        self.model_indices.append(
                            self.model_states.name.index(name))
                    except:
                        continue
                if len(self.model_indices) == len(CYLINDER_NAMES):
                    break
            except:
                print("Waiting for subscribers to be initialized")
                time.sleep(1)

    def pause(self):
        """
        Pause simulation
        """
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            print('Service call failed: %s %e" % (/gazebo/pause_physics, e)')

    def unpause(self):
        """
        Unpause simulation
        """
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            print('Service call failed: %s %e" % (/gazebo/unpause_physics, e)')

    def reset_sim(self):
        """
        Reset simulation
        """
        self.pause()
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            self.reset_sim_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            print('Service call failed: %s %e" % (/gazebo/reset_simulation, e)')

    def reset_env(self):
        """
        Reset environment
        """
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_env_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            print('Service call failed: %s %e" % (/gazebo/reset_world, e)')

    def change_physics(self):
        """
        Setup physic parameters
        """
        self.pause()

        set_physics_request = SetPhysicsPropertiesRequest()
        set_physics_request.time_step = self._time_step
        set_physics_request.max_update_rate = self._max_update_rate
        set_physics_request.gravity = self._gravity
        set_physics_request.ode_config = self._ode_config

        response = self.set_physics(set_physics_request)
        print("Gazebo " + str(response.status_message))
        rospy.loginfo("Gazebo " + str(response.status_message))

        # self.unpause()

    def set_pose(self):
        """
        This Service allows user to set model joint positions without invoking dynamics.
        """

        self.pause()
        set_pose = SetModelConfigurationRequest()
        set_pose.model_name = "robot"
        set_pose.urdf_param_name = ""
        set_pose.joint_names = self.arm_JointNames
        set_pose.joint_positions = [
            1.411872641811951,
            -0.8420041879132294,
            1.0565325644929087,
            -1.7859626190435058,
            -1.5704882948679586,
            1.4109012658608595,
        ]

        rospy.wait_for_service("/gazebo/set_model_configuration")
        try:
            self.set_modelconfig(set_pose)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            print('Service call failed: %s %e" % (/gazebo/set_model_configuration e)')

    def reset_controllers(self):
        """
        Switch controllers off and on again
        """

        rospy.wait_for_service("/controller_manager/switch_controller")
        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = []
            switch_request_object.start_controllers = self.controllers_list
            switch_request_object.strictness = 1
            switch_result = self.switch_service(switch_request_object)
            rospy.logdebug("Switch Result==>" + str(switch_result.ok))
        except rospy.ServiceException as e:
            print(e)

        rospy.wait_for_service("/controller_manager/switch_controller")
        try:
            switch_request_object = SwitchControllerRequest()
            switch_request_object.start_controllers = self.controllers_list
            switch_request_object.start_controllers = []
            switch_request_object.strictness = 1
            switch_result = self.switch_service(switch_request_object)
            rospy.logdebug("Switch Result==>" + str(switch_result.ok))
        except rospy.ServiceException as e:
            print(e)
