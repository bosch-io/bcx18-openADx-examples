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

SHARED_MODEL_PATH=/shared/models
HOME_MODEL_PATH=~/.gazebo/models

mkdir -p $SHARED_MODEL_PATH
mkdir -p $HOME_MODEL_PATH

# provide simulation meshes
if [ ! -d $SHARED_MODEL_PATH/ardupilot_sitl_gazebo_plugin ]; then
    SHA=ea596b3a4259 # branch "master"
    rm -rf *$SHA*
    wget -nv -O $SHA.zip https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin/archive/$SHA.zip
    unzip -q $SHA.zip "*/ardupilot_sitl_gazebo_plugin/*"
    mv *$SHA*/ardupilot_sitl_gazebo_plugin $SHARED_MODEL_PATH
    cp -r $SHARED_MODEL_PATH/ardupilot_sitl_gazebo_plugin/meshes/meshes_sensors/board_camera $SHARED_MODEL_PATH
    touch $SHARED_MODEL_PATH/ardupilot_sitl_gazebo_plugin/model.config
    rm -rf *$SHA*
fi
if [ ! -L $HOME_MODEL_PATH/ardupilot_sitl_gazebo_plugin ]; then
    ln -s -T $SHARED_MODEL_PATH/ardupilot_sitl_gazebo_plugin $HOME_MODEL_PATH/ardupilot_sitl_gazebo_plugin
fi

# provide kinect model with model.config
if [ ! -d $SHARED_MODEL_PATH/kinect ]; then
    SHA=1af392d1445f # tag "tip"
    rm -rf *$SHA*
    wget -nv -O $SHA.zip https://bitbucket.org/osrf/gazebo_models/get/$SHA.zip
    unzip -q $SHA.zip "*/kinect/*"
    mv *$SHA*/kinect $SHARED_MODEL_PATH
    rm -rf *$SHA*
fi
if [ ! -L $HOME_MODEL_PATH/kinect ]; then
    ln -s -T $SHARED_MODEL_PATH/kinect $HOME_MODEL_PATH/kinect
fi

# Download Gazebo models
# git clone https://github.com/erlerobot/erle_gazebo_models
cd $HOME_MODEL_PATH
wget -qO- https://github.com/erlerobot/erle_gazebo_models/archive/bcaef649ad.tar.gz | tar xz && mv -n erle_gazebo_models-*/* .
rm -rf erle_gazebo_models-*
