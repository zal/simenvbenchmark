#!/usr/bin/env python3

from random import uniform

# Number of spheres per side of cube
gridsize = 6
# Radius
r = 0.1
# Mass
m = 10
# Distance between sphere origins
dtr = 2 * r * 1.05
# Pertubation of sphere position in grid
ep = 1e-3
# Offset of entire cube (height)
z_offset = 1


def createWorld(filepath, header, end_str, sim=0):
    ball_id = 0

    world = open(filepath, 'w')
    world.write(header)
    for x in range(gridsize):
        for y in range(gridsize):
            for z in range(gridsize):
                dt = uniform(dtr * (1 - ep), dtr * (1 + ep))
                if sim == 0:
                    world.write(get_webots_string(ball_id,
                                                  r,
                                                  m,
                                                  x * dt,
                                                  y * dt,
                                                  z * dt + z_offset))
                elif sim == 1:
                    world.write(get_pybullet_string(ball_id,
                                                    r,
                                                    m,
                                                    x * dt,
                                                    y * dt,
                                                    z * dt + z_offset))
                elif sim == 2:
                    world.write(get_gazebo_string(ball_id,
                                                  r,
                                                  m,
                                                  x * dt,
                                                  y * dt,
                                                  z * dt + z_offset))

                ball_id += 1

    world.write(end_str)
    world.close()
    print("nnnSim file writen to %s" % filepath)


def get_webots_string(ball_id, r, m, x, y, z):
    targetSphereString = """
    DEF ball_%s Ball {
        translation %f %f %f
        rotation 1 1 1 1
        radius %f
        mass %f
        centerOfMass [
            0 0 0
        ]
            name "ball_%s"
        linearDamping 0.0
        angularDamping 0.0
    }
    """ % (ball_id, x, y, z, r, m, ball_id)
    return targetSphereString


def get_pybullet_string(ball_id, r, m, x, y, z):
    targetSphereString = """
        <body name="ball_%s" pos="%f %f %f"><geom type="sphere" size="%f" mass="%f" rgba=".7 .3 .3 1"/></body>
    """ % (ball_id, x, y, z, r, m)
    return targetSphereString


def get_gazebo_string(ball_id, r, m, x, y, z):
    targetSphereString = """	<model name='ball_%s'>
      <pose>%f %f %f 0 0 0</pose>
      <link name='link'>
        <inertial>
          <mass>%f</mass>
          <pose>0 0 0 0 0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <sphere>
              <radius>%f</radius>
            </sphere>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode>
                <mu>0</mu>
                <mu2>0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <sphere>
              <radius>%f</radius>
            </sphere>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Green</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    """ % (ball_id, x, y, z, m, r, r)
    return targetSphereString


if __name__ == '__main__':
    header_webots = """#VRML_SIM R2021a utf8
    WorldInfo {
      basicTimeStep 1
      physicsDisableLinearThreshold 0.1
      physicsDisableAngularThreshold 0.1
      randomSeed 1
      contactProperties [
        ContactProperties {
          coulombFriction [
            0
          ]
          bounce 0
          softERP 0
          softCFM 1e-12
        }
      ]
    }
    Viewpoint {
      orientation -0.37220907837548917 -0.43373683065992874 -0.8205685612451584 4.36862663557948
      position 9.000484046328555 2.1487996545325165 6.210177115077761
    }
    Background {
      skyColor [
        0.7 0.7 0.7
      ]
    }
    DirectionalLight {
      ambientIntensity 1
      direction 0.1 -0.5 0.3
    }
    RectangleArena {
      translation 0 0.75 0.02
      rotation 1 0 0 1.5707996938995747
      floorSize 100 100
      wallHeight 0.04
    }
    Robot {
      controller "<extern>"
      supervisor TRUE
    }"""

    end_str_webots = ""

    header_pybullet = """<mujoco model="benchmark_case">
    <compiler inertiafromgeom='auto' angle='radian' eulerseq='zyx'/>
    <size njmax="5000" nconmax="1000" />
    <option timestep='0.005' iterations='50' solver='PGS' gravity='0 0 -9.81'/>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
        <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>

        <material name='MatPlane' reflectance='0.1' texture="texplane" texrepeat="1 1" texuniform="true"/>
    </asset>


    <worldbody>
        <geom name="floor" pos="0 0 0" size="0 0 .25" type="plane" material="MatPlane" condim="3"/>"""

    end_str_pybullet = """</worldbody>
</mujoco>"""

    header_gazebo = """<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    """

    end_str_gazebo = """    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>9.27602 4.36925 4.45041 0 0.303643 -2.78299</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
    """

    createWorld("environments/PyBullet/environment/nnnSim.xml",
                header_pybullet, end_str_pybullet, sim=1)
    createWorld("environments/WeBots/worlds/nnnSim.wbt",
                header_webots, end_str_webots, sim=0)
    createWorld("environments/MuJoCo/environment/nnnSim.xml",
                header_pybullet, end_str_pybullet, sim=1)
    createWorld("environments/Gazebo/environment/worlds/nnnSim.world",
                header_gazebo, end_str_gazebo, sim=2)
