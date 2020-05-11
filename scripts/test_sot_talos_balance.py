#! /usr/bin/env python
import sys
import rospy
import rospkg
import time
import unittest
import math
from os.path import abspath, dirname, join

from std_srvs.srv import *
from dynamic_graph_bridge_msgs.srv import *

from gazebo_msgs.srv import *

PKG_NAME='talos_integration_tests'

'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sot_talos_balance.utils.run_test_utils import  \
    run_ft_calibration, run_test, runCommandClient

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
lpath = rospack.get_path(PKG_NAME)
print(lpath)
appli_file_name = join(dirname(abspath(__file__)), 'appli_dcmZmpControl_file.py')

time.sleep(2)
rospy.loginfo("Stack of Tasks launched")

test_folder = 'TestKajita2003StraightWalking64/20cm'
print('Using folder ' + test_folder)

rospy.loginfo("Waiting for run_command")
rospy.wait_for_service('/run_command')
rospy.loginfo("...ok")

runCommandClient('test_folder = "' + test_folder + '"')

run_test(appli_file_name,verbosity=1,interactive=False)
time.sleep(5)
# Connect ZMP reference and reset controllers
print('Connect ZMP reference')

runCommandClient('plug(robot.zmp_estimator.emergencyStop,robot.cm.emergencyStop_zmp)')
runCommandClient('plug(robot.dcm_control.zmpRef,robot.com_admittance_control.zmpDes)')
runCommandClient('robot.com_admittance_control.setState(robot.wp.comDes.value,[0.0,0.0,0.0])')
runCommandClient('robot.com_admittance_control.Kp.value = Kp_adm')
runCommandClient('robot.dcm_control.resetDcmIntegralError()')
runCommandClient('robot.dcm_control.Ki.value = Ki_dcm')

print('Executing the trajectory')
#time.sleep(1)
runCommandClient('robot.triggerTrajGen.sin.value = 1')
time.sleep(25)
runCommandClient('dump_tracer(robot.tracer)')

#input("Wait before check the output")
    
