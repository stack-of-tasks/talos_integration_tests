#! /usr/bin/env python
import sys
import rospy
import time
from os.path import abspath, dirname, join


from std_srvs.srv import *
from dynamic_graph_bridge_msgs.srv import *

PKG_NAME='talos_integration_tests'

runCommandClient = rospy.ServiceProxy('run_command', RunCommand)

def launchScript(code,title,description = ""):
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line != '' and line[0] != '#':
            print line
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print answer
        rospy.loginfo("...done with "+title)

def runTest():
    # Waiting for services
    try:
        rospy.loginfo("Waiting for run_command")
        rospy.wait_for_service('/run_command')
        rospy.loginfo("...ok")

        rospy.loginfo("Waiting for start_dynamic_graph")
        rospy.wait_for_service('/start_dynamic_graph')
        rospy.loginfo("...ok")

        rospy.loginfo("Waiting for /gazebo/get_link_state")
        rospy.wait_for_service('/gazebo/get_link_state')
        rospy.loginfo("...ok")

        runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph',
                                                         Empty)

        # get the file path for rospy_tutorials
        initCode = open(join(dirname(abspath(__file__)), 'appli.py'), "r").read().split("\n")

        rospy.loginfo("Stack of Tasks launched")

        launchScript(initCode,'initialize SoT')
        runCommandStartDynamicGraph()
        runCommandClient("target = (0.5,-0.2,1.0)")
        runCommandClient("gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))")
        runCommandClient("sot.push(taskRH.task.name)")

        time.sleep(10)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    runTest()
