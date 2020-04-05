# Talos integration-tests

This project is for integration tests on Talos aka TALOS-001.

# Setup
```
mkdir -p test_ws/src
cd tests_ws/src
catkin_init_workspace
git clone --recursive https://github.com/stack-of-tasks/talos_integration_tests.git
catkin_make_install
```

Preparing your environment variables:
```
source ./test_ws/install/setup.bash
source $HOME/bin/setup-opt-robotpkg.sh
```
# First integration tests

## Kinematic SoT on gazebo

To launch:
```
rostest talos_integration_tests test_kine.test
```
The robot is supposed to move forward its right hand up to position [0.5723,-0.2885,0.7745]

There is a you tube video showing what to expect:

[![Alt text](http://i3.ytimg.com/vi/gptPEm5Qj3o/hqdefault.jpg)](https://youtu.be/gptPEm5Qj3o)

## Balancing on gazebo

To launch:
```
rostest talos_integration_tests test_sot_talos_balance.test
```
The robot is supposed to walk forward 2.8 m and reached position [2.8331,0.0405,1.0019]

There is a you tube video showing what to expect:

[![Alt text](http://i3.ytimg.com/vi/Hd46shZ22dM/hqdefault.jpg)]((https://youtu.be/Hd46shZ22dM)

