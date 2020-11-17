#!/usr/bin/env python
# O. Stasse 17/01/2020
# LAAS, CNRS

import math
import time
import unittest

import roslaunch
import rospkg
import rospy
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty

PKG_NAME = 'talos_integration_tests'


class TestSoTTalos(unittest.TestCase):
    def validation_through_gazebo(self):
        gzGetModelPropReq = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        gzGetModelPropResp = gzGetModelPropReq(model_name='talos')
        f = open("/tmp/output.dat", "w+")
        f.write("x:" + str(gzGetModelPropResp.pose.position.x) + "\n")
        f.write("y:" + str(gzGetModelPropResp.pose.position.y) + "\n")
        f.write("z:" + str(gzGetModelPropResp.pose.position.z) + "\n")
        # dx depends on the timing of the simulation
        # which can be different from one computer to another.
        # Therefore check only dy and dz.
        dx = 0.0
        dy = gzGetModelPropResp.pose.position.y - 0.0038
        dz = gzGetModelPropResp.pose.position.z - 1.00152
        ldistance = math.sqrt(dx * dx + dy * dy + dz * dz)
        f.write("dist:" + str(ldistance))
        f.close()
        if ldistance < 0.05:
            self.assertTrue(True, msg="Converged to the desired position")
        else:
            self.assertFalse(True, msg="Did not converged to the desired position")

    def runTest(self):
        # Start roscore
        import subprocess
        roscore = subprocess.Popen('roscore')
        time.sleep(1)

        # Get the path to talos_data
        arospack = rospkg.RosPack()
        talos_data_path = arospack.get_path('talos_data')

        # Start talos_gazebo
        rospy.init_node('starting_talos_gazebo', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = [
            talos_data_path + '/launch/talos_gazebo_alone.launch', 'world:=empty_forced', 'enable_leg_passive:=false'
        ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        launch_gazebo_alone = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch_gazebo_alone.start()
        rospy.loginfo("talos_gazebo_alone started")

        rospy.wait_for_service("/gazebo/pause_physics")
        gazebo_pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        gazebo_pause_physics()

        time.sleep(5)
        # Spawn talos model in gazebo
        launch_gazebo_spawn_hs = roslaunch.parent.ROSLaunchParent(
            uuid, [talos_data_path + '/launch/talos_gazebo_spawn_hs.launch'])
        # launch_gazebo_spawn_hs = roslaunch.parent.ROSLaunchParent(uuid,
        #    [ros_data_path+'/launch/talos_gazebo_spawn_hs_wide.launch'])
        launch_gazebo_spawn_hs.start()
        rospy.loginfo("talos_gazebo_spawn_hs started")

        rospy.wait_for_service("/gains/arm_left_1_joint/set_parameters")
        time.sleep(5)
        gazebo_unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        gazebo_unpause_physics()

        # Start roscontrol
        launch_bringup = roslaunch.parent.ROSLaunchParent(uuid, [talos_data_path + '/launch/talos_bringup.launch'])

        launch_bringup.start()
        rospy.loginfo("talos_bringup started")

        # Start sot
        roscontrol_sot_talos_path = arospack.get_path('roscontrol_sot_talos')
        launch_roscontrol_sot_talos = roslaunch.parent.ROSLaunchParent(
            uuid, [roscontrol_sot_talos_path + '/launch/sot_talos_controller_gazebo.launch'])
        launch_roscontrol_sot_talos.start()
        rospy.loginfo("roscontrol_sot_talos started")

        time.sleep(5)
        pkg_name = 'talos_integration_tests'
        executable = 'test_online_walking.py'
        node_name = 'test_online_walking_py'
        test_sot_online_walking_node = roslaunch.core.Node(pkg_name, executable, name=node_name)

        launch_test_sot_online_walking = roslaunch.scriptapi.ROSLaunch()
        launch_test_sot_online_walking.start()

        test_sot_online_walking_process = launch_test_sot_online_walking.launch(test_sot_online_walking_node)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Test if sot_online_walking is finished or not
            if not test_sot_online_walking_process.is_alive():

                self.validation_through_gazebo()

                # If it is finished then find exit status.
                if test_sot_online_walking_process.exit_code != 0:
                    exit_status = "test_online_walking failed"
                    self.assertFalse(True, exit_status)
                else:
                    exit_status = None

                print("Stopping SoT")
                launch_roscontrol_sot_talos.shutdown()
                print("Stopping Gazebo")
                launch_gazebo_alone.shutdown()

                rospy.signal_shutdown(exit_status)

                # Terminate the roscore subprocess
                print("Stop roscore")
                roscore.terminate()

            r.sleep()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG_NAME, 'test_online_walking', TestSoTTalos)
