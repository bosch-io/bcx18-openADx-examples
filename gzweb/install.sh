#!/bin/bash -ex

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

# download and unpack gzweb
cd $HOME
wget -qO- https://bitbucket.org/osrf/gzweb/get/gzweb_1.4.0.tar.gz | tar xz
mv -n osrf-gzweb-* gzweb

# install required packages
sudo apt-get install -y nodejs nodejs-legacy npm libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

# build everything
source source_simulation_environment.bash
source /usr/share/gazebo-7/setup.sh
cd gzweb
./deploy.sh -m local

# add rover model to gzweb
cp -a /shared/models/ardupilot_sitl_gazebo_plugin http/client/assets/
cp /vagrant/gzweb/model.config http/client/assets/ardupilot_sitl_gazebo_plugin/
gz sdf -p /shared/models/ardupilot_sitl_gazebo_plugin/urdf/rover.urdf > http/client/assets/ardupilot_sitl_gazebo_plugin/model.sdf
