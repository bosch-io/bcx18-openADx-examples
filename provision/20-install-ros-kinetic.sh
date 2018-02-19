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

export DEBIAN_FRONTEND=noninteractive

# cf. http://wiki.ros.org/kinetic/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ros-latest.list -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
sudo apt-get install -y ros-kinetic-desktop-full

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi

rosdep update
