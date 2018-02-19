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

# set locale
sudo sh -c 'echo "LANG=en_US.UTF-8\nLC_ALL=en_US.UTF-8" > /etc/default/locale'

if [ "$VBOX_INSTALL_DESKTOP" != '0' ]; then
  # disable screen lock
  export DISPLAY=:0
  export $(dbus-launch)
  gsettings set org.gnome.desktop.screensaver ubuntu-lock-on-suspend false
  gsettings set org.gnome.desktop.lockdown disable-lock-screen true

  # set keyboard layouts
  gsettings set org.gnome.desktop.input-sources sources "[('xkb', 'de'), ('xkb', 'us')]"
fi
