# vlma: Visual Lidar Map Alignment

[![ROS](https://img.shields.io/badge/ROS-noetic-blue)](https://github.com/BEAMRobotics/beam_slam)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-purple)](https://github.com/BEAMRobotics/beam_slam)

## Description:

This repository implements automatic map alignment using visual and lidar topics from ros bags. The only inputs are 2 trajectory files computed from any SLAM package of choice, and the associated bags with their visual and lidar topics. The output will be the trajectory of the second map aligned to the first.

## Dependencies:

* [libbeam](https://github.com/BEAMRobotics/libbeam) An install script is found [here](https://github.com/BEAMRobotics/libbeam/blob/master/scripts/install.bash)

## Results:

Before Alignment            |  After Alignment
:-------------------------:|:-------------------------:
![](https://github.com/jakemclaughlin6/jakemclaughlin6/assets/25440002/62acc213-0652-4563-bd99-86005a45f677)  |  ![](https://github.com/jakemclaughlin6/jakemclaughlin6/assets/25440002/15605e65-9d25-4fbe-8231-6c78022ba7fa)
![](https://github.com/jakemclaughlin6/jakemclaughlin6/assets/25440002/90e93842-c322-4f4a-9234-8d082e8ae903)  |  ![](https://github.com/jakemclaughlin6/jakemclaughlin6/assets/25440002/9c9f7094-b612-4cad-95af-7d52fcd630a6)
