#!/usr/bin/env bash

path="./results/$(date +"test_set_%Y-%m-%d_%H-%M-%S")"

# ---------- Hardware Logging ---------
mkdir -p $path
path=$(realpath $path)
collectl --all -i 0.1 -f $path &
PID_LOGGING=$!
echo "Hardware logging process started with PID=$PID_LOGGING"

# ---------- xvfb --------------------
echo "Starting the virtual X frame buffer"
Xvfb :99 -screen 0 1024x768x16 &
PID_XVFB=$!
DISPLAY=:99

# ---------- WEBOTS ------------------
echo "Starting WeBots..."
${WEBOTS_HOME:-/usr/local/webots}/webots --stdout --stderr --batch --no-sandbox --mode=fast ./environments/WeBots/worlds/TestCase1.wbt &
PID_WEBOTS=$!
echo "WeBots started with PID=$PID_WEBOTS"
python3 ./instanz_manager.py --path=$path --sim="webots"
kill $PID_WEBOTS

sleep 30

echo "Starting WeBots..."
${WEBOTS_HOME:-/usr/local/webots}/webots --stdout --stderr --batch --no-sandbox --mode=fast ./environments/WeBots/worlds/nnnSim.wbt &
PID_WEBOTS=$!
echo "WeBots started with PID=$PID_WEBOTS"
python3 ./instanz_manager.py --path=$path --sim="webots" --sim_option="nnnSim"
kill $PID_WEBOTS

echo "Killing Xvfb"
kill $PID_XVFB
unset DISPLAY

sleep 30

# ---------- PYBULLET ------------------
echo "Starting PyBullet..."
python3 ./instanz_manager.py --path=$path --sim="pybullet"

sleep 30

echo "Starting PyBullet..."
python3 ./instanz_manager.py --path=$path --sim="pybullet" --sim_option="nnnSim"

sleep 30

# ---------- MuJoCo ------------------
echo "Starting MuJoCo..."
python3 ./instanz_manager.py --path=$path --sim="mujoco"

sleep 30

echo "Starting MuJoCo..."
python3 ./instanz_manager.py --path=$path --sim="mujoco" --sim_option="nnnSim"

sleep 30

# ---------- GAZEBO ------------------
echo "Starting Gazebo..."
source ./environments/Gazebo/catkin_ws/devel/setup.bash
launchfile=$(realpath environments/Gazebo/environment/launch/benchmark.launch)
roslaunch $launchfile &
PID_GAZEBO=$!
echo "ROS-Server started with PID=$PID_GAZEBO"
sleep 10
python3 ./instanz_manager.py --path=$path --sim="gazebo"

kill $PID_GAZEBO

sleep 30

echo "Starting Gazebo..."
source ./environments/Gazebo/catkin_ws/devel/setup.bash
launchfile=$(realpath environments/Gazebo/environment/launch/nnnSim.launch)
roslaunch $launchfile &
PID_GAZEBO=$!
echo "ROS-Server started with PID=$PID_GAZEBO"
sleep 10
python3 ./instanz_manager.py --path=$path --sim="gazebo" --sim_option="nnnSim"

kill $PID_GAZEBO

# Stops the hardware logging and saves the log file
kill $PID_LOGGING

sleep 30

# ---------- Hardware Monitoring ------------------
# Convert raw file to plot file
monitor_file=$(find $path -type f -name '*.gz')
echo "Converting hardware monitor file: $monitor_file"

collectl -sCmD -o Tm -p $monitor_file >> ${monitor_file::-3}

sleep 10

# Hardware data post-processing
python3 ./hardware_monitoring/hardware_data_postprocessor.py --path=$path
