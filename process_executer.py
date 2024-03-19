import subprocess as p
import time

def run_in_terminal(command):
    p.Popen(["gnome-terminal", "--", "bash", "-c", command])
    time.sleep(5)


roscore = run_in_terminal("roscore")

rplidar = run_in_terminal("roslaunch rplidar_ros rplidar.launch")

mavros = run_in_terminal("roslaunch mavros apm.launch")

scancorrect = run_in_terminal("rosrun scan_correction scan_correction_node")

hectorslam = run_in_terminal("roslaunch hector_slam_launch tutorial.launch")
