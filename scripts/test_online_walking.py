#! /usr/bin/env python
import sys
import rospy
import rospkg
import time
import unittest
import math

from std_srvs.srv import *
from dynamic_graph_bridge_msgs.srv import *

from gazebo_msgs.srv import *

PKG_NAME='talos_integration_tests'

'''Test online walking pattern generator'''

from sys import argv
from sot_talos_balance.utils.run_test_utils import \
    run_ft_calibration, run_test, runCommandClient

from time import sleep

def wait_for_dynamic_graph():
    try:
        rospy.loginfo("Waiting for run_command")
        rospy.wait_for_service('/run_command')
        rospy.loginfo("...ok")

        rospy.loginfo("Waiting for start_dynamic_graph")
        rospy.wait_for_service('/start_dynamic_graph')
        rospy.loginfo("...ok")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def run_code(appli):
    initCode = open(appli, "r").read()
    print(dir(initCode))
    print(initCode)
    out=runCommandClient(initCode)
    print(out)
    if (out.standardoutput or out.standarderror):
        if out.standardoutput:
            print("standardoutput: " + out.standardoutput)
        if out.standarderror:
            print("standarderror: " + out.standarderror)
    return out


# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get an instance of RosPack with the default search paths
lpath = rospack.get_path(PKG_NAME)
print(lpath)

appli_file_name =lpath + '/../../lib/'+PKG_NAME+\
    '/appli_online_walking.py'


runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph',Empty)
rospy.loginfo("Stack of Tasks launched")

wait_for_dynamic_graph()
rospy.loginfo("Wait before running the code")
#run_code(appli_file_name)
run_test(appli_file_name,verbosity=1,interactive=False)
rospy.loginfo("Wait before starting the dynamic graph")
runCommandStartDynamicGraph()

rospy.loginfo("Stack of Tasks launched")


#run_test(appli_file_name, verbosity=1,interactive=False)
time.sleep(5)

# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
runCommandClient(
    'plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient(
    'plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient(
    'robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])'
)
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

print('Executing the trajectory')
runCommandClient('robot.triggerPG.sin.value = 1')

time.sleep(4)
runCommandClient('robot.pg.velocitydes.value=(0.2,0.0,0.0)')
time.sleep(7)
runCommandClient('robot.pg.velocitydes.value=(0.3,0.0,0.0)')
time.sleep(9)
runCommandClient('robot.pg.velocitydes.value=(0.0,0.0,0.0)')

time.sleep(5)
runCommandClient('dump_tracer(robot.tracer)')
