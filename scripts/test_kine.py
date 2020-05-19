#! /usr/bin/env python
import sys
import rospy
import time
from os.path import abspath, dirname, join


from std_srvs.srv import *
from dynamic_graph_bridge_msgs.srv import *

runCommandClient = rospy.ServiceProxy('run_command', RunCommand)

from sot_talos_balance.utils.run_test_utils import runCommandClient

def handleRunCommandClient(code):
    out = runCommandClient(code)

    if out.standarderror:
        print("standarderror: " + out.standarderror)
        sys.exit(-1)

PKG_NAME='talos_integration_tests'

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

        rospy.loginfo("Stack of Tasks launched")

        handleRunCommandClient('from talos_integration_tests.appli import init_appli')
        handleRunCommandClient('init_sot_talos_balance(robot)')

        launchScript(initCode,'initialize SoT')
        handleRunCommandStartDynamicGraph()
        handleRunCommandClient("target = (0.5,-0.2,1.0)")
        handleRunCommandClient("gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))")
        handleRunCommandClient("sot.push(taskRH.task.name)")

        time.sleep(10)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    runTest()
