#!/bin/bash -eu

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

# The drcsim package is not available for Ubuntu 16.04.
# We there have to remove references to this from launch files.

ROVER_LAUNCH="~/rover-simulation/ros_catkin_ws/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/rover_spawn.launch"

function help {
    echo -e "Usage: `basename $0` [-h] [launchfile]\n"
    echo "Patch launch files like '$ROVER_LAUNCH'."
}

if [ $1 == "-h" ]; then
  help
  exit 0
fi

DRCSIM_REFERENCE="\$(find drcsim_model_resources)\/gazebo_models\/environments:"
LAUNCH_FILE=$1

if [ -e $LAUNCH_FILE ]; then
    echo "Removing '$DRCSIM_REFERENCE' from '$LAUNCH_FILE'."
    sed -i.backup 's/'"$DRCSIM_REFERENCE"'//' "$LAUNCH_FILE"
else
    echo "The file '$LAUNCH_FILE' does not exist."
    help
    exit 1
fi
