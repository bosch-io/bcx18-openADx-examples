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

# In Gazebo simulations where Gazebo server and client run on different machines
# model resources referenced by ROS package are not found on the client side
# unless they lie at exactly the same location as referenced on the server,
# cf. http://answers.gazebosim.org/question/15618/gzclient-tries-to-load-mesh-with-absolute-filename/
# We therefore rather reference them by Gazebo model path which we may directly control on the client.


ROVER_URDF="~/rover-simulation/ros_catkin_ws/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/urdf/rover.urdf"

function help {
    echo -e "Usage: `basename $0` [-h] [urdffile]\n"
    echo "Patch URDF files like '$ROVER_URDF'."
}

if [ $1 == "-h" ]; then
  help
  exit 0
fi

URDF_FILE=$1

if [ -e $URDF_FILE ]; then
    echo "Replacing 'package' by 'model' in '$URDF_FILE'."
    sed -i.backup 's#package://ardupilot_sitl_gazebo_plugin#model://ardupilot_sitl_gazebo_plugin#g' "$URDF_FILE"
else
    echo "The file '$URDF_FILE' does not exist."
    help
    exit 1
fi
