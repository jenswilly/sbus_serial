# sbus_serial

ROS 2 package for parsing Futaba SBUS serial data from a RC receiver for remote control (teleoperation) of ROS-based robots.

SBUS is a serial protocol for RC receivers where the values of up to 16 channels are sent over one serial channel.

Read the [Wiki](https://github.com/jenswilly/sbus_serial/wiki/ROS2) which contains all the information you need to use this package.

The package contains the following executables:

- `sbus_serial_node` containing:
  - `sbus_serial`: node reading SBUS data from a serial port and publishing `Sbus` data
- `sbus_cmd_vel_node` containing:
  - `sbus_cmd_vel`: node subscribing to the `/sbus` topic and publishing corresponding `Twist` _or_ `TwistStamped` message on the `/output/sbus/cmd_vel` topic. Use parameters and topic remapping as required. (See comments in sbus_cmd_vel_node.cpp for more info)
- `sbus_calibrate` (installed in the packages's `share` directory): stand-alone executable to calibrate data from the receiver (see details in the [Wiki](https://github.com/jenswilly/sbus_serial/wiki/ROS2)).

The package also contains the following custom interfaces:

- `sbus_serial/msg/Sbus`:  
  Values received and mapped from the SBUS input.

## ROS version

The code in this branch is written for [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html).

The [ros1](https://github.com/jenswilly/sbus_serial/tree/ros1) branch contains the old code for ROS 1.

## Compiling

Compile as usual: `colcon build`.

If you get an error saying, "boost/algorithm/clamp.hpp: No such file or directory", you need to install the Boost C++ libraries. On Ubuntu, run `sudo apt-get install libboost-dev`.

## Usage

A typical setup will launch `sbus_serial` and `sbus_cmd_vel` nodes.

The `sbus_serial` node should be configured with port name (e.g. `/dev/ttyS0`) and appropriate values for min/max (see the [Wiki](https://github.com/jenswilly/sbus_serial/wiki/ROS2)).

The `sbus_cmd_vel` node should be configured with Sbus min/max matching the output from the `sbus_serial` node
and appropriate min/max values for `Twist` speed and turn. The published topic (`/output/sbus/cmd_vel`) should
also be remapped to the appropriate topic.

### Turtlesim

The package contains a launch file to use the Sbus to control the Turtlesim for testing.

The launch file launches `turtlesim turtlesim_node` and `sbus_serial sbus_cmd_vel_node` but _not_ the `sbus_serial_node` as that could be running on another device (e.g. the SBUS RC is connected to a Raspberry Pi controller and the Turtlesim
is running on a full desktop Ubuntu computer). So be sure to launch `sbus_serial sbus_serial_node` with appropriate parameters separately or the test will be real boring.

The `sbus_serial_node` should output Sbus values from 0-255 and channel 3 (throttle) is used for forward speed while channel 1 (aileron) is used for turning.

If the `minSpeed` parameter is set to a negative value, the turtle will reverse when control stick is at bottom.
