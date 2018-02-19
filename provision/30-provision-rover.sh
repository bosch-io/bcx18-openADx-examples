#!/bin/bash -eux

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

# cf. http://docs.erlerobotics.com/simulation/configuring_your_environment
# The documentation describes an installation for Ubuntu 14.04 with ROS Indigo.
# With some modifications the build works for Ubuntu 16.04 with ROS Kinetic.

SIMULATION_DIR=$HOME/rover-simulation
SIMULATION_ZIP=/shared/rover-simulation.tar.bz2
GAZEBO_DEB_DIR=/shared/gazebo7

# Base packages
sudo apt-get install -y gawk make git curl cmake unzip

# MAVProxy dependencies
# python-wxgtk2.8 is not available for Ubuntu 16.04, use python-wxgtk3.0 instead
sudo apt-get install -y \
     g++ \
     python-pip \
     python-matplotlib \
     python-serial \
     python-wxgtk3.0 \
     python-scipy \
     python-opencv \
     python-numpy \
     python-pyparsing \
     ccache \
     realpath \
     libopencv-dev

# Install MAVProxy
sudo -H pip install --upgrade pip
sudo -H pip install future
sudo apt-get install -y libxml2-dev libxslt1-dev
sudo -H pip install pymavlink==2.2.8 catkin_pkg==0.4.1 --upgrade
sudo -H pip install MAVProxy==1.5.2

# Additional ROS Kinetic dependencies
sudo apt-get install -y \
     python-rosinstall \
     ros-kinetic-octomap-msgs \
     ros-kinetic-joy \
     ros-kinetic-geodesy \
     ros-kinetic-octomap-ros \
     ros-kinetic-mavlink \
     ros-kinetic-control-toolbox \
     ros-kinetic-transmission-interface \
     ros-kinetic-joint-limits-interface

# Install Gazebo 7.9
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu xenial main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget -nv http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/gazebo-latest.list -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y libignition-math2=2.9.0-1~xenial libignition-math2-dev=2.9.0-1~xenial libsdformat4=4.4.0-1~xenial libsdformat4-dev=4.4.0-1~xenial sdformat-sdf=6.0.0-1~xenial libbullet-dev libbulletsoftbody2.83.6
if [[ ! -d $GAZEBO_DEB_DIR ]]; then
    mkdir -p $GAZEBO_DEB_DIR
    wget -nv -P $GAZEBO_DEB_DIR http://gazebosim.org/distributions/gazebo/releases/{gazebo7,gazebo7-plugin-base,libgazebo7,libgazebo7-dev}_7.9.0-1~xenial_amd64.deb
    wget -nv -P $GAZEBO_DEB_DIR http://gazebosim.org/distributions/gazebo/releases/gazebo7-common_7.9.0-1~xenial_all.deb
fi
sudo dpkg -i $GAZEBO_DEB_DIR/*.deb
sudo apt-mark hold gazebo7 libgazebo7-dev libignition-math2 libignition-math2-dev libsdformat4 libsdformat4-dev sdformat-sdf

if [[ -d $SIMULATION_DIR && -e $SIMULATION_ZIP ]]; then
    echo "The directory $SIMULATION_DIR and cache $SIMULATION_ZIP already exist. The rover simulation and its dependencies will therefore not be re-provisioned."
    exit 0
fi

if [[ -e $SIMULATION_ZIP ]]; then
    echo "A pre-compiled (zipped) version of $SIMULATION_DIR was detected in '/shared'. It will be unzipped and installed."
    tar -I pbzip2 -xf $SIMULATION_ZIP --directory $HOME

    # ArUco
    cd $SIMULATION_DIR/aruco-2.0.19/build
    sudo make install
else
    if [[ -d $SIMULATION_DIR ]]; then
        mv $SIMULATION_DIR $SIMULATION_DIR.$(date -Iseconds).bak
    fi
    echo "Neither an existing version of the rover simulation nor a pre-compiled (zipped) version of $SIMULATION_DIR was detected."
    echo "The rover simulation and its dependencies will be completely build anew."
    ./build-rover.sh
    cd
    tar -I pbzip2 -cf $SIMULATION_ZIP rover-simulation
fi
