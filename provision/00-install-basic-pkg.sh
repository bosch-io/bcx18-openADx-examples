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

sudo apt-get update
sudo apt-get -y upgrade

if [ "$VBOX_INSTALL_DESKTOP" != '0' ]; then
  sudo apt-get -y install ubuntu-desktop mesa-utils
fi

sudo apt-get -y install \
  zsh \
  emacs24 \
  vim \
  git \
  wget \
  jq \
  python-pip \
  pbzip2 \
  cifs-utils
