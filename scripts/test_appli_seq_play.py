#!/usr/bin/python
import sys
import rospy

from std_srvs.srv import *
from dynamic_graph_bridge_msgs.srv import *


def launchScript(code, title, description=""):
    raw_input(title + ':   ' + description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line != '' and line[0] != '#':
            print line
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print answer
    rospy.loginfo("...done with " + title)


# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    initCode = open("appli_seq_play.py", "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    launchScript(initCode, 'initialize SoT')
    raw_input("Wait before starting the dynamic graph")
    runCommandStartDynamicGraph()

    raw_input("Wait before starting the seqplay")
    runCommandClient("aSimpleSeqPlay.start()")

except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s" % e)
