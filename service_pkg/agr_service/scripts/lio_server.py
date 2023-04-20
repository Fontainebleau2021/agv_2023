#!/usr/bin/env python
import subprocess
import rospy
import rosnode
import roslaunch
import thread,time
from std_srvs.srv import Trigger, TriggerResponse
from agr_service.srv import *

SLAM_Command = 2
navigation_Command = 2


class launch_demo:
    def __init__(self, cmd=None):
        self.cmd = cmd
 
    def launch(self):
        self.child = subprocess.Popen(self.cmd)
        return True
 
    def shutdown(self):
        self.child.terminate()
        self.child.wait()
        return True

def slam_command_thread():
    global SLAM_Command
    launch_slam = launch_demo(["roslaunch", "lio_sam", "run_livox.launch"])  
    while True:
        if SLAM_Command == 1:   
            launch_slam.launch()
            SLAM_Command = 3
        if SLAM_Command == 0:
            launch_slam.shutdown()
            SLAM_Command = 2
        time.sleep(0.1)

def navigation_command_thread():
    global navigation_Command
    launch_start_navigation = launch_demo(["roslaunch", "navigation", "navigation_trans_start_node.launch"])
    launch_lio_start_navigation = launch_demo(["roslaunch", "navigation", "navigation_trans_lio_start_node.launch"])   
    launch_mid_navigation = launch_demo(["roslaunch", "navigation", "navigation_trans_mid_node.launch"])  
    launch_lio_end_navigation = launch_demo(["roslaunch", "navigation", "navigation_trans_lio_end_node.launch"]) 
    while True:
        if navigation_Command == 1:   
            launch_start_navigation.launch()
            navigation_Command = 3
        if navigation_Command == 0:
            launch_start_navigation.shutdown()
            navigation_Command = 2
        if navigation_Command == 5:
            launch_lio_start_navigation.launch()
            navigation_Command = 7
        if navigation_Command == 4:
            launch_lio_start_navigation.shutdown()
            navigation_Command = 6
        if navigation_Command == 9:
            launch_mid_navigation.launch()
            navigation_Command = 11
        if navigation_Command == 8:
            launch_mid_navigation.shutdown()
            navigation_Command = 10
        if navigation_Command == 13:
            launch_lio_end_navigation.launch()
            navigation_Command = 15
        if navigation_Command == 12:
            launch_lio_end_navigation.shutdown()
            navigation_Command = 14
        
        time.sleep(0.1)

def commandCallback(msg):

    global SLAM_Command
    global navigation_Command

    if msg.slam == 1 and SLAM_Command != 3:
        SLAM_Command = 1
    elif msg.slam == 0 and SLAM_Command == 3:
        SLAM_Command = 0

    if msg.navigation == 1 and navigation_Command != 3 and navigation_Command != 7 and navigation_Command != 11 and navigation_Command != 15:
        navigation_Command = 1
    elif msg.navigation == 0 and navigation_Command == 3:
        navigation_Command = 0
    elif msg.navigation == 5 and navigation_Command != 3 and navigation_Command != 7 and navigation_Command != 11 and navigation_Command != 15:
        navigation_Command = 5
    elif msg.navigation == 4 and navigation_Command == 7:
        navigation_Command = 4
    elif msg.navigation == 9 and navigation_Command != 3 and navigation_Command != 7 and navigation_Command != 11 and navigation_Command != 15:
        navigation_Command = 9
    elif msg.navigation == 8 and navigation_Command == 11:
        navigation_Command = 8
    elif msg.navigation == 13 and navigation_Command != 3 and navigation_Command != 7 and navigation_Command != 11 and navigation_Command != 15:
        navigation_Command = 13
    elif msg.navigation == 12 and navigation_Command == 15:
        navigation_Command = 12
    
    rospy.loginfo("Publish command![%d]", SLAM_Command)
    resp = agr_serviceResponse()
    resp.result = "ON"
    return resp

	

def command_server():
    rospy.init_node('command_server')

    s = rospy.Service('/command', agr_service, commandCallback)

    print "Ready to receive command."

    thread.start_new_thread(slam_command_thread, ())
    thread.start_new_thread(navigation_command_thread, ())
    rospy.spin()

if __name__ == "__main__":
    command_server()
 
