#!/usr/bin/env python3

import rospy
import time

from gazebo_msgs.msg import ODEPhysics, ModelStates
from gazebo_msgs.srv import (
    SetPhysicsProperties,
    SetPhysicsPropertiesRequest,
    SetModelConfiguration
)
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty


class nnnEnv_gazebo:
    def __init__(self, render=False):
        # ROS Subscribers
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_states_callback
        )
        self.clock = rospy.Subscriber("/clock", Clock, self.clock_callback)

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

        self.model_states = None
        self.model_states_list = []
        self.sim_time = 0.0
        self.clock = 0.0

        # ----------- Start Timer -----------
        self.sys_start_time = time.time()
        rospy.init_node("RobotEnv_gazebo", anonymous=True,
                        log_level=rospy.WARN)
        self.reset()
        self.t0 = time.time()
        self.t1 = 0
        self.rate = rospy.Rate(1000)

    def execute_n_timesteps(self, n=1):
        return

    def reset(self):
        """
        Reset simulation
        """
        self.reset_sim()
        self.balls_states = [0] * 216
        self.model_states_list = []
        print("reset: ", rospy.get_time(), self.sim_time)
        self.sim_time = 0.0

    def set_timestep(self, timestep):
        """ Setup simulation timestep

        Args:
            timestep (float): simulation timestep
        """
        self._time_step = float(timestep) / 1000.0
        self.change_physics()

    def get_obs(self):
        """ Get observation from simulation

        Returns:
            list: Timer and observation data
        """

        # Time
        sys_time = time.time()
        sys_timer = sys_time - self.sys_start_time

        # balls positions
        for i in range(len(self.balls_states)):
            cyl_pose = self.model_states.pose[i]
            pos = [cyl_pose.position.x, cyl_pose.position.y, cyl_pose.position.z]
            rot = [
                cyl_pose.orientation.w,
                cyl_pose.orientation.x,
                cyl_pose.orientation.y,
                cyl_pose.orientation.z,
            ]
            self.balls_states[i] = [pos, rot]

        return [sys_timer, self.sim_time, self.balls_states]

    def model_states_callback(self, msg):
        self.model_states = msg
        self.model_states_list.append([self.sim_time, msg])

    def clock_callback(self, msg):
        self.sim_time = msg.clock.secs + msg.clock.nsecs * 1e-9

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
