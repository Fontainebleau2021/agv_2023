#!/usr/bin/env python
import subprocess
import rospy
import rosnode
import thread,time
from std_srvs.srv import Trigger, TriggerResponse

pubCommand = 0

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



def command_thread():
    global pubCommand
    while True:
        if pubCommand == 1:
            launch_nav = launch_demo(["roslaunch", "gps_viewer", "map_load.launch"])
            launch_nav.launch()
            pubCommand = 3
        if pubCommand == 0:
            rosnode.kill_nodes(['gps2map','map_load','map_rviz'])
            pubCommand = 2
        time.sleep(0.1)

def commandCallback(req):

    global pubCommand

    if pubCommand == 0 or pubCommand == 2:
        pubCommand = 1
    elif pubCommand == 1 or pubCommand == 3:
        pubCommand = 0
    
	rospy.loginfo("Publish command![%d]", pubCommand)

	return TriggerResponse(1, "Change command state!")

def command_server():
    rospy.init_node('command_server')

    s = rospy.Service('/command', Trigger, commandCallback)

    print "Ready to receive command."

    thread.start_new_thread(command_thread, ())
    rospy.spin()

if __name__ == "__main__":
    command_server()
 
