#!/usr/bin/env python3


import pybullet
import pybullet_data
from pybullet_utils import bullet_client


class PyBulletInterface:
    """
    Base class for Bullet physics simulation environments in a Scene.
    These environments create single-player scenes and behave like normal Gym environments, if
    you don't use multiplayer.
    Based on: https://github.com/benelot/pybullet-gym/blob/master/pybulletgym/envs/roboschool/envs/env_bases.py
    """

    def __init__(self, render):
        self._p = None
        self.isRender = render
        self.physicsClientId = -1
        self.ownsPhysicsClient = 0
        self.robot = None
        self.objects = None

        if self.physicsClientId < 0:
            self.ownsPhysicsClient = True

            if self.isRender:
                self._p = bullet_client.BulletClient(
                    connection_mode=pybullet.GUI)
            else:
                self._p = bullet_client.BulletClient()

            self.physicsClientId = self._p._client
            self._p.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

        self._p.setRealTimeSimulation(0)

    def load_model(self, model_path=None, robot_path=None):
        if model_path is not None:
            self.objects = self._p.loadMJCF(
                model_path
            )  # , flags=pybullet.URDF_USE_SELF_COLLISION|pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)#|pybullet.ACTIVATION_STATE_ENABLE_SLEEPING)

        if robot_path is not None:
            base_position = [0.0, 0.0, 0.02]
            base_orientation = [0, 0, 0, 1]

            self.robot = self._p.loadURDF(
                robot_path,
                basePosition=base_position,
                baseOrientation=base_orientation,
                useFixedBase=1,
                # flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
            )

        return self.objects, self.robot

    def reset(self, model_path, robot_path=None, init_pos=None):
        self._p.resetSimulation()
        self._p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.objects, self.robot = self.load_model(robot_path=robot_path)

        if init_pos is not None or self.robot is not None:
            self.reset_joint_state(self.robot, init_pos)

        if self.robot is not None:
            self.num_joints = self._p.getNumJoints(self.robot)

        self.do_simulation(5)

        self.objects, self.robot = self.load_model(model_path=model_path)

        return self.objects, self.robot

    def set_physic_parameters(self, only_main_setting=False):
        self._p.setGravity(0, 0, -9.81)
        # self._p.setPhysicsEngineParameter(numSolverIterations=10)
        # self._p.setPhysicsEngineParameter(enableConeFriction=0)
        # self._p.setPhysicsEngineParameter(solverResidualThreshold=1e-5)
        # self._p.setPhysicsEngineParameter(collisionFilterMode=0)
        # self._p.setPhysicsEngineParameter(contactBreakingThreshold=0.02)

        if only_main_setting is False:
            self._p.setPhysicsEngineParameter(contactERP=0.2)
            # self._p.setPhysicsEngineParameter(frictionERP=0.1)
            # self._p.setPhysicsEngineParameter(collisionFilterMode=1)
            # self._p.setPhysicsEngineParameter(globalCFM=1e-4)

            if self.objects is not None:
                for o in self.objects:
                    self._p.changeDynamics(o, -1, lateralFriction=0.45)
                    self._p.changeDynamics(o, -1, spinningFriction=0.05)
                    self._p.changeDynamics(o, -1, rollingFriction=0.001)
                    self._p.changeDynamics(o, -1, restitution=0.0)
                    self._p.changeDynamics(o, -1, linearDamping=0.04)
                    self._p.changeDynamics(o, -1, angularDamping=0.04)
                    self._p.changeDynamics(o, -1, frictionAnchor=1)
                    # self._p.changeDynamics(o, -1, activationState=pybullet.ACTIVATION_STATE_ENABLE_SLEEPING)
                    # self._p.changeDynamics(o, -1, activationState=pybullet.ACTIVATION_STATE_WAKE_UP)

            if self.robot is not None:
                self._p.changeDynamics(
                    self.robot, -1, lateralFriction=1)  # 0.45)
                self._p.changeDynamics(self.robot, -1, spinningFriction=0.05)
                self._p.changeDynamics(self.robot, -1, rollingFriction=0.001)
                self._p.changeDynamics(self.robot, -1, restitution=0.0)
                self._p.changeDynamics(self.robot, -1, linearDamping=0.04)
                self._p.changeDynamics(self.robot, -1, angularDamping=0.04)
                self._p.changeDynamics(self.robot, -1, frictionAnchor=1)
        else:
            self._p.setPhysicsEngineParameter(
                contactERP=0.0,
                # solverResidualThreshold=1e-3,
                # collisionFilterMode=0,
            )
            if self.objects is not None:
                for i in range(len(self.objects)):
                    self._p.changeDynamics(
                        self.objects[i], -1, lateralFriction=0.0)
                    self._p.changeDynamics(
                        self.objects[i], -1, spinningFriction=0.0)
                    self._p.changeDynamics(
                        self.objects[i], -1, rollingFriction=0.0)
                    self._p.changeDynamics(
                        self.objects[i], -1, restitution=0.0)
                    self._p.changeDynamics(
                        self.objects[i], -1, linearDamping=0.0)
                    self._p.changeDynamics(
                        self.objects[i], -1, angularDamping=0.0)

                    if i != 0:
                        self._p.changeDynamics(self.objects[i], -1, mass=10)

                    # self._p.changeDynamics(o, -1, contactProcessingThreshold=0)
                    # print(0, self._p.getBodyInfo(o))
                    # print(self._p.getDynamicsInfo(o, -1))

    def get_joint_info(self, body):
        info = []
        for j in range(self._p.getNumJoints(body)):
            # print(self._p.getJointInfo(body, j))
            info.append(self._p.getJointInfo(body, j))

        return info

    def get_body_state(self, bodies):
        # getBasePositionAndOrientation reports the current position
        # and orientation of the base (or root link) of the body in
        # Cartesian world coordinates. The orientation is a quaternion
        # in [x,y,z,w] format.
        cylinder_states = []

        for i in range(len(bodies)):
            cylinder_states.append(
                self._p.getBasePositionAndOrientation(bodies[i]))

        return cylinder_states

    def get_body_positions(self, bodies):
        # getBasePositionAndOrientation reports the current position
        # and orientation of the base (or root link) of the body in
        # Cartesian world coordinates. The orientation is a quaternion
        # in [x,y,z,w] format.
        cylinder_states = []

        for i in range(len(bodies)):
            cylinder_states.append(
                self._p.getBasePositionAndOrientation(bodies[i])[0])

        return cylinder_states

    def get_joint_state(self, obj, target_name=None):
        joint_multi = self._p.getJointStates(
            self.robot, range(self.num_joints))
        return joint_multi

    def set_joint_states_vel(self, body, joint_states):
        joint_len = len(joint_states.keys())
        jointIndices = [None] * joint_len
        targetVelocities = [None] * joint_len
        joint_name = list(joint_states.keys())

        for i in range(len(joint_states.keys())):
            jointIndices[i] = joint_states[joint_name[i]]["joint_index"]
            targetVelocities[i] = joint_states[joint_name[i]]["vel_target"]

        self._p.setJointMotorControlArray(
            bodyIndex=body,
            jointIndices=jointIndices,
            controlMode=self._p.VELOCITY_CONTROL,
            targetVelocities=targetVelocities,
            velocityGains=[1.0] * joint_len,
        )

    def reset_joint_state(self, body, joint_states):
        for joint in joint_states.keys():
            self._p.resetJointState(
                bodyUniqueId=body,
                jointIndex=joint_states[joint]["joint_index"],
                targetValue=joint_states[joint]["target"],
            )

    def set_timestep(self, timestep):
        timestep = timestep / 1000
        self._p.setTimeStep(timestep)

    def step(self, *args, **kwargs):
        self._p.stepSimulation()

    def do_simulation(self, n):
        for _ in range(n):
            self._p.stepSimulation()

    def close(self):
        if self.ownsPhysicsClient:
            if self.physicsClientId >= 0:
                self._p.disconnect()
        self.physicsClientId = -1
