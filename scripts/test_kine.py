#! /usr/bin/env python
import sys
import rospy
import rospkg
import time
import unittest
import math

from std_srvs.srv import *
from dynamic_graph_bridge.srv import *
from dynamic_graph_bridge_msgs.srv import *

from gazebo_msgs.srv import *

PKG='pyrene_integration_tests'

class TestSoTTalos(unittest.TestCase):
    
    def launchScript(self,code,title,description = ""):
        rospy.loginfo(title)
        rospy.loginfo(code)
        for line in code:
            if line != '' and line[0] != '#':
                print line
                answer = self.runCommandClient(str(line))
                rospy.logdebug(answer)
                print answer
        rospy.loginfo("...done with "+title)

    def test_sot_talos(self):
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

            self.runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
            self.runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

            # get an instance of RosPack with the default search paths
            rospack = rospkg.RosPack()

            # get the file path for rospy_tutorials
            lpath = rospack.get_path('pyrene_integration_tests')
            print(lpath)
            initCode = open( lpath + '/../../lib/pyrene_integration_tests/appli.py', "r").read().split("\n")
            
            rospy.loginfo("Stack of Tasks launched")

            self.launchScript(initCode,'initialize SoT')
            self.runCommandStartDynamicGraph()
            self.runCommandClient("target = (0.5,-0.2,1.0)")
            self.runCommandClient("gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))")
            self.runCommandClient("sot.push(taskRH.task.name)")
            
            time.sleep(5)
    
            gzGetLinkPropReq = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
            gzGetLinkPropResp = gzGetLinkPropReq(link_name='gripper_right_fingertip_1_link')
            f=open("/tmp/output.dat","w+")
            f.write("x:"+str(gzGetLinkPropResp.link_state.pose.position.x)+"\n")
            f.write("y:"+str(gzGetLinkPropResp.link_state.pose.position.y)+"\n")
            f.write("z:"+str(gzGetLinkPropResp.link_state.pose.position.z)+"\n")
            dx=gzGetLinkPropResp.link_state.pose.position.x-0.4757
            dy=gzGetLinkPropResp.link_state.pose.position.y+0.3172
            dz=gzGetLinkPropResp.link_state.pose.position.z-0.8010
            ldistance = math.sqrt(dx*dx+dy*dy+dz*dz)
            f.write("dist:"+str(ldistance))
            f.close()
            if ldistance<0.09:
                self.assertTrue(True)
            else:
                self.assertTrue(False)
    
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG,'test_sot_talos',TestSoTTalos)
