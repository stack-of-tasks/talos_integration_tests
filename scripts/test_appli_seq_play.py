#!/usr/bin/python

import rospy
from dynamic_graph_bridge_msgs.srv import RunCommand
from std_srvs.srv import Empty

try:
    # Python 2
    input = raw_input
except NameError:
    pass


def launchScript(code, title, description=""):
    input(title + ':   ' + description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line and not line.strip().startswith('#'):
            print(line)
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print(answer)
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
    input("Wait before starting the dynamic graph")
    runCommandStartDynamicGraph()

    input("Wait before starting the seqplay")
    runCommandClient("aSimpleSeqPlay.start()")

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s" % e)
