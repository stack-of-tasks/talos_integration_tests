ARG UBUNTU_VERSION=18.04
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu${UBUNTU_VERSION}

ARG UBUNTU=bionic
ARG ROS_DISTRO=melodic

# ROS & robotpkg packages repository
ENV ROS_KEY=C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 ROBOTPKG_KEY=F6F93D4D425860C0B0FBE848ADD535E05E56C3FD
ADD http://repo.ros2.org/repos.key http://robotpkg.openrobots.org/packages/debian/robotpkg.key /
RUN apt-get update -qqy && apt-get install -qqy gnupg2 && rm -rf /var/lib/apt/lists \
 && echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU} main" > /etc/apt/sources.list.d/ros-latest.list \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub ${UBUNTU} robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub ${UBUNTU} robotpkg" >> /etc/apt/sources.list.d/robotpkg.list \
 && export GNUPGHOME=$(mktemp -d) \
 && gpg2 --import-options import-show /repos.key | grep -q $ROS_KEY \
 && gpg2 --import-options import-show /robotpkg.key | grep -q $ROBOTPKG_KEY \
 && rm -rf $GNUPGHOME \
 && apt-key add /repos.key /robotpkg.key

# environment helpers
ENV ROS_PREFIX=/opt/ros/${ROS_DISTRO} ROBOTPKG_BASE=/opt/openrobots
# required environment variables
ENV PYTHONPATH=${ROBOTPKG_BASE}/lib/python2.7/site-packages:${ROS_PREFIX}/lib/python2.7/dist-packages \
    CMAKE_PREFIX_PATH=${ROBOTPKG_BASE}:${ROS_PREFIX} \
    LD_LIBRARY_PATH=${ROBOTPKG_BASE}/lib/dynamic-graph-plugins \
    QT_X11_NO_MITSHM=1

# talos-integration-tests dependencies
RUN apt-get update -qqy \
 && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    git \
    python-catkin-tools \
    python-rosdep \
    robotpkg-py27-talos-dev \
 && rm -rf /var/lib/apt/lists

# catkin workspace
WORKDIR /ws
RUN rosdep init \
 && rosdep update \
 && ${ROS_PREFIX}/bin/catkin_init_workspace

# talos-integration-tests build
ADD . /ws/src/talos_integration_tests
RUN catkin config --install \
 && catkin build talos_integration_tests

ENTRYPOINT ["/ws/src/talos_integration_tests/entrypoint.sh"]
CMD ["/bin/bash"]
