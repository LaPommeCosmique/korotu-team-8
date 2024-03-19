import subprocess as p
import time

def run_in_terminal(command):
    p.Popen(["gnome-terminal", "--", "bash", "-c", command])


roscore = run_in_terminal("roscore")

time.sleep(5)

talker = run_in_terminal("roslaunch rplidar_ros rplidar.launch")

time.sleep(1)

mavros = run_in_terminal("roslaunch mavros apm.launch")

time.sleep(1)

scancorrect = run_in_terminal("rosrun scan_correction scan_correction_node")

time.sleep(1)

hectorslam = run_in_terminal("roslaunch hector_slam_launch tutorial.launch")