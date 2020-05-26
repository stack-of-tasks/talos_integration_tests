ARG UBUNTU_VERSION=18.04
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu${UBUNTU_VERSION}

ARG UBUNTU=bionic
ARG ROS_DISTRO=melodic

# ROS packages
RUN apt-get update -qqy && DEBIAN_FRONTEND=noninteractive apt-get install -qqy gnupg2 && rm -rf /var/lib/apt/lists \
 && echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU} main" > /etc/apt/sources.list.d/ros-latest.list \
 && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# robotpkg packages
ADD http://robotpkg.openrobots.org/packages/debian/robotpkg.key /
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub ${UBUNTU} robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub ${UBUNTU} robotpkg" >> /etc/apt/sources.list.d/robotpkg.list \
 && apt-key add /robotpkg.key

# environment helpers
ENV ROS_PREFIX=/opt/ros/${ROS_DISTRO} ROBOTPKG_BASE=/opt/openrobots
# required environment variables
ENV PYTHONPATH=/ws/src/talos_integration_tests/src:${ROBOTPKG_BASE}/lib/python2.7/site-packages:${ROS_PREFIX}/lib/python2.7/dist-packages \
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
RUN catkin build talos_integration_tests
