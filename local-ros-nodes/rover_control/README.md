# Rover Control example ROS node

Copyright (c) 2018 Bosch Software Innovations GmbH.
All rights reserved. This program and the accompanying materials
are made available under the terms of the Eclipse Public License v2.0
which accompanies this distribution, and is available at
http://www.eclipse.org/legal/epl-v20.html

Contributors: Bosch Software Innovations GmbH - initial creation

*Disclaimer: This software is experimental and intended to be used on the Bosch Connected Experience 2018*

## Enable control via ROS

In order to be able to control the rover via a ROS node, you have to set the (MAV-) parameter `SYSID_MYGCS` to `1` once:

```bash
rosrun mavros mavparam set SYSID_MYGCS 1
```

## How to control the rover

You can steer the rover with two parameters, throttle and steering (yaw). Both parameters vary in the range from 1100 to 1900.

   Range    |      Yaw       |    Throttle
:---------: | :------------: | :------------:
1100 - 1500 |   turn left    | drive backward
   1500     | drive straight |      stop
1500 - 1900 |   turn right   | drive forward

Every steering command must be sent to the topic `/mavros/rc/override` as an array of eight channels. Channel 0 corresponds to yaw, channel 2 to throttle:

```bash
rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn '[yaw, 0, throttle, 0, 0, 0, 0, 0]'
```

or

```python
message = OverrideRCIn()
message.channels = [yaw, 0, throttle, 0, 0, 0, 0, 0]
rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10).publish(message)
```

## How to read sensor data

In order to read data from sensors, you have to subscribe to topics:
```bash
rostopic echo /topic/name
```

or

```python
def call_back_function(data):
  # react to incoming message

rospy.Subscriber(topicName, topicType, call_back_function)
```

To get a list of all available topics, use
```bash
rostopic list -v
```

See [Example](http://docs.erlerobotics.com/simulation/vehicles/erle_rover/tutorial_1)


## Helpful links

[Examples for navigating the rover and analyzing sensor data](https://github.com/erlerobot/gazebo_python_examples)

## Remote execution

To run this node on an other host than the ROS master, you have to set the variable

```bash
export ROS_IP=IP.OF.THE.NODE
export ROS_MASTER_URI=http://IP.OF.THE.MASTER:11311/
```

and on the master:

```bash
export ROS_IP=IP.OF.THE.MASTER
```

**Note:** Do not set `ROS_HOSTNAME` unless you set up a correct name resolution. `ROS_HOSTNAME` and `ROS_IP` are mutually exclusive (and the former wins). See: [ROS wiki](http://wiki.ros.org/action/fullsearch/ROS/EnvironmentVariables)
