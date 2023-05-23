# vl_map_refinement

## Description:

This repo implements trajectory refinement using typical visual reprojection constraints and tightly coupled visual-lidar constraints, given an initial trajectory estimate (from odometry or SLAM). It has been tested on Ubuntu 18.04 with ROS Melodic.

## Dependencies:

* fuse: https://github.com/locusrobotics/fuse (devel branch for ROS Melodic)
* libbeam: https://github.com/BEAMRobotics/libbeam and all of it's dependencies. An install script is dfound at libbeam/scripts/install.bash

## Compiling:

To compile, put this repo, fuse, and tf2_2d into the same catkin workspace. Optionally put libbeam in the same space or compile and install globally using make. Then run `catkin build vl_map_refinement` in the catkin workspace.

The main file is under devel/lib/vl_map_refinement, to run perform: `./vl_map_refinement_main --config_file [absolute path to config file]`

When compiling a conflict emerges with the rosbag.h file, the solution can be found [here](https://github.com/ethz-asl/lidar_align/issues/16)