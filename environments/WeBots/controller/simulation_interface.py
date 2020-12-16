#!/usr/bin/env python3


from controller import Supervisor


class WebotsInterface():
    def __init__(self):
        self.sim_interface = Supervisor()
        self.root = self.sim_interface.getRoot()
        self.WorldInfo = self.root.getField('children').getMFNode(0)
        self.timestep = int(self.sim_interface.getBasicTimeStep())
        self.NodeTypes = [
            'NO_NODE',
            'APPEARANCE',
            'BACKGROUND',
            'BOX',
            'CAPSULE',
            'COLOR',
            'CONE',
            'COORDINATE',
            'CYLINDER',
            'DIRECTIONAL_LIGHT',
            'ELEVATION_GRID',
            'FOG',
            'GROUP',
            'IMAGE_TEXTURE',
            'INDEXED_FACE_SET',
            'INDEXED_LINE_SET',
            'MATERIAL',
            'MUSCLE',
            'NORMAL',
            'PBR_APPEARANCE',
            'PLANE',
            'POINT_LIGHT',
            'POINT_SET',
            'SHAPE',
            'SPHERE',
            'SPOT_LIGHT',
            'TEXTURE_COORDINATE',
            'TEXTURE_TRANSFORM',
            'TRANSFORM',
            'VIEWPOINT',
            'ROBOT',
            'DIFFERENTIAL_WHEELS',
            'ACCELEROMETER',
            'BRAKE',
            'CAMERA',
            'COMPASS',
            'CONNECTOR',
            'DISPLAY',
            'DISTANCE_SENSOR',
            'EMITTER',
            'GPS',
            'GYRO',
            'INERTIAL_UNIT',
            'LED',
            'LIDAR',
            'LIGHT_SENSOR',
            'LINEAR_MOTOR',
            'PEN',
            'POSITION_SENSOR',
            'PROPELLER',
            'RADAR',
            'RANGE_FINDER',
            'RECEIVER',
            'ROTATIONAL_MOTOR',
            'SPEAKER',
            'TOUCH_SENSOR',
            'BALL_JOINT',
            'BALL_JOINT_PARAMETERS',
            'CHARGER',
            'CONTACT_PROPERTIES',
            'DAMPING',
            'FLUID',
            'FOCUS',
            'HINGE_JOINT',
            'HINGE_JOINT_PARAMETERS',
            'HINGE_2_JOINT',
            'IMMERSION_PROPERTIES',
            'JOINT_PARAMETERS',
            'LENS',
            'LENS_FLARE',
            'PHYSICS',
            'RECOGNITION',
            'SLIDER_JOINT',
            'SLOT',
            'SOLID',
            'SOLID_REFERENCE',
            'TRACK',
            'TRACK_WHEEL',
            'WORLD_INFO',
            'MICROPHONE',
            'RADIO',
            'SKIN']

    def execute_n_timesteps(self, n=1):
        """ Performing simulation step(s)

        Args:
            n (int, optional): Amount of simulation steps to be performed. Defaults to 1.
        """

        self.sim_interface.step(n * self.timestep)

    def resetSim(self):
        """ Reset simulation """
        self.sim_interface.simulationReset()

    def get_devices(self):
        """ Get information about all devices in Webots world """

        n = self.sim_interface.getNumberOfDevices()
        devices = []
        device_names = []
        device_types = []

        for i in range(n):
            device = self.sim_interface.getDeviceByIndex(i)
            devices.append(device)
            device_names.append(device.getName())
            device_types.append(self.NodeTypes[device.getNodeType()-1])

        return devices, device_names, device_types
