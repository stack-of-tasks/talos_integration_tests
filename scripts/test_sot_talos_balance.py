#! /usr/bin/env python
import sys
import time
from os.path import abspath, dirname, join

import rospy
from sot_talos_balance.utils.run_test_utils import runCommandClient
from std_srvs.srv import Empty


def handleRunCommandClient(code):
    out = runCommandClient(code)

    if out.standarderror:
        print("standarderror: " + out.standarderror)
        sys.exit(-1)


PKG_NAME = 'talos_integration_tests'
'''Test CoM admittance control as described in paper, with pre-loaded movements'''

# get the file path for rospy_tutorials
appli_file_name = join(dirname(abspath(__file__)), 'appli_dcmZmpControl_file.py')

time.sleep(2)
rospy.loginfo("Stack of Tasks launched")

test_folder = 'TestKajita2003StraightWalking64/20cm'
print('Using folder ' + test_folder)

rospy.loginfo("Waiting for run_command")
rospy.wait_for_service('/run_command')
rospy.loginfo("...ok")

handleRunCommandClient('test_folder = "' + test_folder + '"')

handleRunCommandClient('from talos_integration_tests.appli_dcmZmpControl_file import init_sot_talos_balance')
handleRunCommandClient('init_sot_talos_balance(robot,\'' + test_folder + '\')')
time.sleep(5)
runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

runCommandStartDynamicGraph()
time.sleep(5)
# Connect ZMP reference and reset controllers
print('Connect ZMP reference')
handleRunCommandClient('from dynamic_graph import plug')
handleRunCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
handleRunCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
handleRunCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
handleRunCommandClient('Kp_adm = [15.0, 15.0, 0.0]')  # this value is employed later
handleRunCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
handleRunCommandClient('robot.dcm_control.resetDcmIntegralError()')
handleRunCommandClient('Ki_dcm = [1.0, 1.0, 1.0]')  # this value is employed later
handleRunCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

print('Executing the trajectory')
time.sleep(1)
handleRunCommandClient('robot.triggerTrajGen.sin.value = 1')
time.sleep(25)
