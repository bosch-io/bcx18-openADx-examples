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

if [ "$VBOX_INSTALL_DESKTOP" != '0' ]; then
  sudo sh -c 'echo "[Seat:*]\nautologin-user=vagrant" > /etc/lightdm/lightdm.conf'

  sudo systemctl restart lightdm
  sudo systemctl daemon-reload
fi
