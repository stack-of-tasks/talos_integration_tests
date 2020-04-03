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

'''Test CoM admittance control as described in paper, with pre-loaded movements'''
from sot_talos_balance.utils.run_test_utils import ask_for_confirmation, \
    run_ft_calibration, run_test, runCommandClient

class TestSoTTalosBalance(unittest.TestCase):
    
    def test_sot_talos(self):
        # Waiting for services
        try:

            # get an instance of RosPack with the default search paths
            rospack = rospkg.RosPack()

            # get the file path for rospy_tutorials
            lpath = rospack.get_path(PKG_NAME)
            print(lpath)
            appli_file_name =lpath + '/../../lib/'+PKG_NAME+\
                             '/appli_dcmZmpControl_file.py'

            time.sleep(2)
            rospy.loginfo("Stack of Tasks launched")

            test_folder = 'TestKajita2003StraightWalking64/20cm'
            print('Using folder ' + test_folder)
            
            rospy.loginfo("Waiting for run_command")
            rospy.wait_for_service('/run_command')
            rospy.loginfo("...ok")

            runCommandClient('test_folder = "' + test_folder + '"')
            
            run_test(appli_file_name,verbosity=1,interactive=False)
            time.sleep(10)
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
            gzGetModelPropReq = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
            gzGetModelPropResp = gzGetModelPropReq(model_name='talos')
            f=open("/tmp/output.dat","w+")
            f.write("x:"+str(gzGetModelPropResp.pose.position.x)+"\n")
            f.write("y:"+str(gzGetModelPropResp.pose.position.y)+"\n")
            f.write("z:"+str(gzGetModelPropResp.pose.position.z)+"\n")
            dx=gzGetModelPropResp.pose.position.x-2.8331
            dy=gzGetModelPropResp.pose.position.y-0.0405
            dz=gzGetModelPropResp.pose.position.z-1.0019
            ldistance = math.sqrt(dx*dx+dy*dy+dz*dz)
            f.write("dist:"+str(ldistance))
            f.close()
            if ldistance<0.009:
                self.assertTrue(True,msg="Converged to the desired position")
            else:
                self.assertFalse(False,
                                 msg="Did not converged to the desired position")
    
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG_NAME,'test_sot_talos_balance',TestSoTTalosBalance)
