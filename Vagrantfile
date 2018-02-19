# -*- mode: ruby -*-
# vi: set ft=ruby :

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

require 'etc'

cpu_count = ENV['VBOX_CPUCOUNT'] || [(Etc.nprocessors - 1), 4].min
mem_size = ENV['VBOX_MEMSIZE'] || 8192

puts <<EOF
Assigning #{cpu_count} CPUs and #{mem_size}MB to VM.
To override, set the environment variables VBOX_CPUCOUNT and VBOX_MEMSIZE.
EOF

Vagrant.configure('2') do |config|
  plugins = %w( vagrant-vbguest vagrant-disksize )
  restart_vagrant = false
  plugins.each do |plugin|
    if (! Vagrant.has_plugin? plugin)
      system "vagrant plugin install #{plugin}"
      restart_vagrant = true
    end
  end

  if (restart_vagrant)
    exec 'vagrant ' + ARGV.join(' ')
  end

  config.vm.box = 'ubuntu/xenial64'

  config.disksize.size = '120GB'

  config.vm.synced_folder 'shared/', '/shared'

  config.vm.network "private_network", ip: "192.168.50.4"

  config.vm.provider 'virtualbox' do |vb|
    vb.name = 'BCX DevBox'
    vb.gui = true
    vb.customize ['modifyvm', :id, '--memory', mem_size]
    vb.customize ['modifyvm', :id, '--cpus', cpu_count]
    vb.customize ['modifyvm', :id, '--graphicscontroller', 'vboxvga']
    vb.customize ['modifyvm', :id, '--accelerate3d', 'off']
    vb.customize ['modifyvm', :id, '--vram', 256]
    vb.customize ['modifyvm', :id, '--uartmode1', 'disconnected']
    vb.customize ['modifyvm', :id, '--clipboard', 'bidirectional']
  end

  envvars = {
    'VBOX_INSTALL_DESKTOP' => ENV['VBOX_INSTALL_DESKTOP'] || 1,
  }

  # cache apt packages if vagrant-cachier plugin available
  envvars['VBOX_CACHIER_PRESENT'] = 0
  if Vagrant.has_plugin?('vagrant-cachier')
    config.cache.scope = :box
    envvars['VBOX_CACHIER_PRESENT'] = 1
  end

  config.vm.provision 'file', source: 'vm-scripts', destination: '$HOME'
  config.vm.provision 'shell', privileged: false, inline: 'chmod +x $HOME/*.*sh'
  config.vm.provision 'file', source: 'config/tmux.conf', destination: '$HOME/.tmux.conf'
  config.vm.provision 'file', source: 'tmuxinator', destination: '$HOME/.tmuxinator'
  config.vm.provision 'file', source: 'local-ros-nodes', destination: '$HOME/local-ros-nodes'
  config.vm.provision 'file', source: 'eclipse-hono', destination: '$HOME/eclipse-hono'


  Dir.glob('./provision/*.sh').sort.each do |file|
    config.vm.provision 'shell', privileged: false, path: file, env: envvars
  end
end
