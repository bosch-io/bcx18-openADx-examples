#!/usr/bin/env python
 
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

import rospy
import sys

from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import Image


## Make sure to run
# rosrun mavros mavparam set SYSID_MYGCS 1
# once.

## If you want to run this node remotely, set the variable
# ROS_IP to the IP address the node will run on.
# Note: do not set ROS_HOSTNAME
nodeName = 'rover_control'

roverControlTopic = '/mavros/rc/override'
camera_image_topic = '/rover/front/image_front_raw'

rc = rospy.Publisher(roverControlTopic, OverrideRCIn, queue_size=10)

def rover_control(yaw = 1500, throttle = 1500):
  if change_mode('manual'):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        update(rc, yaw, throttle)
        rate.sleep()

def change_mode(mode):
    rospy.wait_for_service('/mavros/set_mode')
    service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    return service(custom_mode = mode)

def new_rc_msg(yaw, throttle):
    message = OverrideRCIn()
    message.channels = [yaw, 0, throttle, 0, 0, 0, 0, 0]
    return message

def update(rc, yaw, throttle):
    rc.publish(new_rc_msg(yaw, throttle))

def on_new_image(data):
    return; # react to incoming data

def rover_stop():
    rover_control(1500, 1500)

if __name__ == '__main__':
    print("Exit with Ctrl-C")
    try:
        rospy.init_node(nodeName, anonymous=True)
        camera = rospy.Subscriber(camera_image_topic, Image, on_new_image)
        if len(sys.argv) != 3:
            rover_stop()
        else:
            yaw = int(sys.argv[1])
            throttle = int(sys.argv[2])
            rover_control(yaw, throttle)
    except rospy.ROSInterruptException:
        pass
