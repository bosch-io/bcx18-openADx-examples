#!/bin/bash

# Copyright (c) 2018 Bosch Software Innovations GmbH.
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
#
# Contributors:
#    Bosch Software Innovations GmbH - initial creation
#
# Disclaimer: This software is experimental and intended to be used on the Bosch Connected Experience 2018

# cf. https://github.com/ros2/ros2/wiki/Linux-Development-Setup
# cf. https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source

set -exo pipefail
export DEBIAN_FRONTEND=noninteractive

ROS2_DIR=~/ros2_ws
SIMULATION_DIR=~/rover-simulation

NUM_CPU=1

# install build tools
sudo apt-get update
sudo apt-get install -y git wget
sudo apt-get install -y \
     build-essential \
     cppcheck \
     cmake \
     libopencv-dev \
     python-empy \
     python3-dev \
     python3-empy \
     python3-nose \
     python3-pip \
     python3-pyparsing \
     python3-setuptools \
     python3-vcstool \
     python3-yaml \
     libtinyxml-dev \
     libeigen3-dev

# dependencies for testing
sudo apt-get install -y \
     clang-format \
     pydocstyle \
     pyflakes \
     python3-coverage \
     python3-mock \
     python3-pep8 \
     uncrustify

# Install argcomplete for command-line tab completion from the ROS2 tools.
# Install from pip rather than from apt-get because of a bug in the Ubuntu 16.04 version of argcomplete:
sudo pip3 install argcomplete

# additional testing dependencies from pip (because not available on ubuntu 16.04)
sudo pip3 install \
     flake8 \
     flake8-blind-except \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     pytest \
     pytest-cov \
     pytest-runner

# dependencies for FastRTPS
sudo apt-get install -y libasio-dev libtinyxml2-dev

# dependencies for RViz
sudo apt-get install -y \
     libcurl4-openssl-dev \
     libqt5core5a \
     libqt5gui5 \
     libqt5opengl5 \
     libqt5widgets5 \
     libxaw7-dev \
     libgles2-mesa-dev \
     libglu1-mesa-dev \
     qtbase5-dev

if [ -d $ROS2_DIR ]; then
    echo "The directory $ROS2_DIR already exists. Please delete it manually if you would like to build it anew."
    exit 1 
fi

# Get ROS 2.0 code, create workspace and clone all repos
# Take current master instead of release-latest
mkdir -p $ROS2_DIR/src
cd $ROS2_DIR
wget -nv https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs-import src < ros2.repos

# Build from source (skip ROS 1 bridge)
# Recommendation: not have ROS 1 environment sourced
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --skip-packages ros1_bridge

# Source ROS 1 environment
source /opt/ros/kinetic/setup.bash

# Source ROS 2 environment
source $ROS2_DIR/install/local_setup.bash

# Source simulation environment
if [ -e $SIMULATION_DIR/ros_catkin_ws/devel/setup.bash ]; then
    source $SIMULATION_DIR/ros_catkin_ws/devel/setup.bash 
fi

# Build ROS 1 bridge (limit number of parallel jobs to 1)
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --only ros1_bridge --force-cmake-configure -j$NUM_CPU
