# Talos integration-tests

This project is for integration tests on Pyrene aka TALOS-001.
The main target of this package is to test the functional level of various software packages.
The API to access the robot are compatible. What is not currently tested is the overall CPU bandwith.
The adequation between the real system and the one currently simulated depends on the current limitation
of the algorithm and the weights used to in the simulator.

In this specific context PID gains for position control are not the one provided by PAL-robotics.
They are much more high and provide a more rigid behavior that was found experimentally closer to reality for some part
of the robot. However, for now the flexibility in the hip found in real TALOS humanoid robots is not simulated.

# Setup
```
mkdir -p test_ws/src
cd tests_ws/src
catkin_init_workspace
git clone --recursive https://github.com/stack-of-tasks/talos_integration_tests.git
catkin build talos_integration_tests
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

[![Alt text](http://i3.ytimg.com/vi/Hd46shZ22dM/hqdefault.jpg)](https://youtu.be/Hd46shZ22dM)

## Real time on-line walking on gazebo

To launch:
```
rostest talos_integration_tests test_online_walking.test
```
The robot is supposed to walk forward 2.8 m and reached position [2.12,0.012,1.00]


# Docker

```
xhost +local:
# 18.04 / melodic
docker build -t talos-integration-tests:18.04 .
docker run --rm --net=host --runtime=nvidia -e DISPLAY -it talos-integration-tests:18.04
# 16.04 / kinetic
docker build -t talos-integration-tests:16.04 --build-arg UBUNTU_VERSION=16.04 --build-arg UBUNTU=xenial --build-arg ROS_DISTRO=kinetic .
docker run --rm --net=host --runtime=nvidia -e DISPLAY -it talos-integration-tests:16.04
```

Once in the container:
```
source devel/setup.bash
rostest talos_integration_tests test_kine.test
rostest talos_integration_tests test_sot_talos_balance.test
```
