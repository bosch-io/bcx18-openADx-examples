#!/bin/bash

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

set -exo pipefail

export DEBIAN_FRONTEND=noninteractive

# cf. http://docs.erlerobotics.com/simulation/configuring_your_environment
# The documentation describes an installation for Ubuntu 14.04 with ROS Indigo.
# With some modifications the build works for Ubuntu 16.04 with ROS Kinetic.

SIMULATION_DIR=$HOME/rover-simulation
SIMULATION_CATKIN_WS=$SIMULATION_DIR/ros_catkin_ws

# Rover Workspace
if [ -d $SIMULATION_DIR ]; then
    echo "The directory $SIMULATION_DIR already exists. Please delete it manually if you would like to build it anew."
    exit 1
fi
mkdir -p $SIMULATION_DIR
cd $SIMULATION_DIR

# ArUco
# 1.3.0 fails, so try the latest versions for which a CMake build is available
wget -nv -N https://sourceforge.net/projects/aruco/files/2.0.19/aruco-2.0.19.zip
unzip aruco-2.0.19.zip
cd aruco-2.0.19/
mkdir build && cd build
cmake ..
make
sudo make install

# APM/Ardupilot
cd $SIMULATION_DIR
# git clone https://github.com/erlerobot/ardupilot -b gazebo
wget -qO- https://github.com/erlerobot/ardupilot/archive/fe72403259.tar.gz | tar xz && mv ardupilot-* ardupilot

# Assume ROS is already installed
source /opt/ros/kinetic/setup.bash

# Catkin Workspace
mkdir -p $SIMULATION_CATKIN_WS/src
cd $SIMULATION_CATKIN_WS/src
catkin_init_workspace
cd $SIMULATION_CATKIN_WS
catkin_make
source devel/setup.bash

# Clone various repositories
cd src/
# git clone https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin
wget -qO- https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin/archive/ea596b3a42.tar.gz | tar xz && mv ardupilot_sitl_gazebo_plugin-* ardupilot_sitl_gazebo_plugin
# git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/ -b kinetic-devel
wget -qO- https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/archive/f9f6e4362a.tar.gz | tar xz && mv hector_gazebo-* hector_gazebo
# git clone https://github.com/erlerobot/rotors_simulator -b sonar_plugin
wget -qO- https://github.com/erlerobot/rotors_simulator/archive/89c722f771.tar.gz | tar xz && mv rotors_simulator-* rotors_simulator
# git clone https://github.com/PX4/mav_comm.git
wget -qO- https://github.com/PX4/mav_comm/archive/5e0a51c242.tar.gz | tar xz && mv mav_comm-* mav_comm
# git clone https://github.com/ethz-asl/glog_catkin.git
wget -qO- https://github.com/ethz-asl/glog_catkin/archive/82db5ffcad.tar.gz | tar xz && mv glog_catkin-* glog_catkin
# git clone https://github.com/catkin/catkin_simple.git
wget -qO- https://github.com/catkin/catkin_simple/archive/0e62848b12.tar.gz | tar xz && mv catkin_simple-* catkin_simple
# git clone https://github.com/erlerobot/mavros.git
wget -qO- https://github.com/erlerobot/mavros/archive/f450e132de.tar.gz | tar xz && mv mavros-* mavros
# git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel
wget -qO- https://github.com/ros-simulation/gazebo_ros_pkgs/archive/a3e69a05c4.tar.gz | tar xz && mv gazebo_ros_pkgs-* gazebo_ros_pkgs

# git clone https://github.com/ros-controls/ros_control.git -b kinetic-devel
wget -qO- https://github.com/ros-controls/ros_control/archive/81ed9edf2a.tar.gz | tar xz && mv ros_control-* ros_control

# Add Python and C++ examples
# git clone https://github.com/erlerobot/gazebo_cpp_examples
wget -qO- https://github.com/erlerobot/gazebo_cpp_examples/archive/f46527e12f.tar.gz | tar xz && mv gazebo_cpp_examples-* gazebo_cpp_examples
# git clone https://github.com/erlerobot/gazebo_python_examples
wget -qO- https://github.com/erlerobot/gazebo_python_examples/archive/6e819123db.tar.gz | tar xz && mv gazebo_python_examples-* gazebo_python_examples

# Compile only messages
cd $SIMULATION_CATKIN_WS
catkin_make --pkg mav_msgs mavros_msgs gazebo_msgs

# Use patches
cp $SIMULATION_CATKIN_WS/src/glog_catkin/fix-unused-typedef-warning.patch $SIMULATION_CATKIN_WS/src/
cp $SIMULATION_CATKIN_WS/src/glog_catkin/get-newest-config-guess.sh $SIMULATION_CATKIN_WS/src/

# Use patches for launch and urdf files
ROVER_LAUNCH="$SIMULATION_CATKIN_WS/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/rover_spawn.launch"
ROVER_MAZE_LAUNCH="$SIMULATION_CATKIN_WS/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/rover_maze.launch"
ROVER_LINE_FOLLOWER_LAUNCH="$SIMULATION_CATKIN_WS/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/rover_circuit.launch"
ROVER_URDF="$SIMULATION_CATKIN_WS/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/urdf/rover.urdf"
bash ~/patch_launch_file.sh $ROVER_LAUNCH
bash ~/patch_launch_file.sh $ROVER_MAZE_LAUNCH
bash ~/patch_launch_file.sh $ROVER_LINE_FOLLOWER_LAUNCH
bash ~/patch_urdf_file.sh $ROVER_URDF

# Patch launcher script to work without X-based terminals (it has timing problems)
cat > $SIMULATION_DIR/ardupilot/Tools/autotest/run_in_terminal_window.sh <<'EOF'
# patched by build-rover.sh

name=$1
cmd=$2
shift
shift

( sleep 3s; $cmd $* > /tmp/${name}-$(date +%Y%m%d-%H%M%S).log < /dev/null )&
exit 0
EOF

# Enable C++11 build
sed -i 's/add_executable/add_definitions(-std=c++11)\nadd_executable/g' $SIMULATION_CATKIN_WS/src/gazebo_cpp_examples/erle-copter_pattern_follower/CMakeLists.txt

# Build workspace
source devel/setup.bash
catkin_make
