# Robotiq

It is based on the original Robotiq repository and it has been updated to
ROS Noetic.

The (old) implemented models (only 3f so far) were extended so that they work
with newly implemented effort_controllers in Gazebo 11.

#### TODO:

* PID fine-tuning for 3f-gripper controller
* Implement 2f-gripper model-extension for Gazebo 11
* Implement 2f-gripper controller

#### Usage 3f-gripper:

For test purposes and as a stand-alone, use the launchfile with a fixed Gripper:

    cd robotiq_3f_gripper_articulated_gazebo/launch
    roslaunch fixed_robotiq_gripper.launch
    
Additionally, use a Python-script to test the movement:

    cd ../control-test
    python gripper_test.py
    
For integration into bigger environments (e.g. with Robot-arms) create a new
package with a high-level .xacro which combines all parts of the new robot.

Use `robotiq-3f-gripper_articulated_macro_transmissions.xacro`, implement it as
child to the robot-arm and add the ros_control plugin,
see `fxed_robotiq-3f-gripper_articulated.xacro` as an example.

#### Usage 2f-gripper:

tbd