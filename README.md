kvh_gyroscope
=============

This repo holds the ROS driver for the KVH DSP Fibre Optic Gyroscope that can be found in one of the OSL vehicles.

Guidelines
----------

Each driver should be created in an indipended ROS package with the same name of the driver. The use of ROS stacks has been deprecated with the introduction of the `catkin` build system and its use is discouraged. Initially this repo is providing packages using the `rosbuild` build system until the OSL vehicles are migrated to an upgraded version of the ROS. Later the `rosbuild` support is going to be dropped and the `master` branch will offer a _catkinized_ package format.
