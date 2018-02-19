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

wget -nv http://repo.ros2.org/repos.key -O - | sudo apt-key add -

sudo sh -c 'echo "deb http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt-get update -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ros2-latest.list -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"

sudo apt-get install -y ros-ardent-*

sudo -H pip install argcomplete
