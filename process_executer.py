import subprocess as p
import time

def run_in_terminal(command):
    p.Popen(["gnome-terminal", "--", "bash", "-c", command])


roscore = run_in_terminal("roscore")

time.sleep(5)

rplidar = run_in_terminal("roslaunch rplidar_ros rplidar.launch")

time.sleep(5)

mavros = run_in_terminal("roslaunch mavros apm.launch")

time.sleep(5)

scancorrect = run_in_terminal("rosrun scan_correction scan_correction_node")

time.sleep(5)

hectorslam = run_in_terminal("roslaunch hector_slam_launch tutorial.launch")
