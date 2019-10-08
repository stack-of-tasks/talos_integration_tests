# Pyrene integration-tests

This project is for integration tests on Pyrene aka TALOS-001.

# Setup
```
mkdir -p test_ws/src
cd tests_ws/src
catkin_init_workspace
git clone --recursive git@gepgitlab.laas.fr:pyrene-dev/pyrene_integration_tests.git
catkin_make_install
```

Preparing your environment variables:
```
source ./test_ws/install/setup.bash
source $HOME/bin/setup-opt-robotpkg.sh
```
# First integration tests.
Kinematic SoT on gazebo.

To launch:
```
rostest pyrene_integration_tests test_kine.test
```

