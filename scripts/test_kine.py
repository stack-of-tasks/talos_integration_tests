#! /usr/bin/env python
import sys
import time

import rospy
from dynamic_graph_bridge_msgs.srv import RunCommand
from std_srvs.srv import Empty

runCommandClient = rospy.ServiceProxy('run_command', RunCommand)


def handleRunCommandClient(code):
    out = runCommandClient(code)

    if out.standarderror:
        print("standarderror: " + out.standarderror)
        sys.exit(-1)


PKG_NAME = 'talos_integration_tests'


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

        runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

        rospy.loginfo("Stack of Tasks launched")

        handleRunCommandClient('from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d')
        handleRunCommandClient(
            'robot.taskRH  = MetaTaskKine6d(\'rh\',robot.dynamic,\'rh\',robot.OperationalPointsMap[\'right-wrist\'])')
        handleRunCommandClient('from dynamic_graph.sot.core.sot import SOT')
        handleRunCommandClient('robot.sot = SOT(\'sot\')')

        handleRunCommandClient('from talos_integration_tests.appli import init_appli')

        handleRunCommandClient('init_appli(robot)')

        handleRunCommandClient('from dynamic_graph.sot.core.meta_tasks_kine import gotoNd')
        runCommandStartDynamicGraph()
        handleRunCommandClient("target = (0.5,-0.2,1.0)")
        handleRunCommandClient("gotoNd(robot.taskRH,target,'111',(4.9,0.9,0.01,0.9))")
        handleRunCommandClient("robot.sot.push(robot.taskRH.task.name)")

        time.sleep(10)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    runTest()
