# vlma

## Description:

This repository implements automatic trajectory alignment using visual and lidar topics from ros bags. The only inputs are 2 trajectory files computed from any SLAM package of choice, and the associated bags with their visual and lidar topics. The output will be the trajectory of the second map aligned to the first.

## Dependencies:

* libbeam: https://github.com/BEAMRobotics/libbeam and all of it's dependencies. An install script is found at libbeam/scripts/install.bash
