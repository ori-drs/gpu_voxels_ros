gpu_voxels_ros
===================================================

## Overview
-----

This package is used as an interface to the GPU-Voxels library, publishing <em>distance field</em> information to be used in motion planning. 
In particular we use the gpu_voxels_hsr_server, live_composite_sdf, and single_composite_sdf classes.

## Install Instructions
-----
This package can be built using catkin build within a catkin workspace. Please ensure that the environment variable GPU_VOXELS_INSTALL_DIR is set to your GPU-Voxels install directory.
## License
-----
While this repository is licensed under the BSD-3-Clause license, it uses two files which are licensed under Apache License, Version 2.0 (Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland and adapted from Paul Furgale Schweizer Messer)

## Citing
-----

If you use this work, please cite following publications:

```
@INPROCEEDINGS{Finean2022,
  author={Finean, Mark Nicholas and Petrović, Luka and Merkt, Wolfgang and Marković, Ivan and Havoutis, Ioannis},
  title={Motion Planning in Dynamic Environments Using Human Trajectory Prediction}, 
  year={2022},
  }

@inproceedings{Finean2021,
author = {Finean, Mark Nicholas and Merkt, Wolfgang and Havoutis, Ioannis},
booktitle = {2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
doi = {10.1109/IROS51168.2021.9636860},
eprint = {2103.03958},
isbn = {978-1-6654-1714-3},
month = {sep},
pages = {3710--3717},
publisher = {IEEE},
title = {{Simultaneous Scene Reconstruction and Whole-Body Motion Planning for Safe Operation in Dynamic Environments}},
url = {http://arxiv.org/abs/2103.03958 https://ieeexplore.ieee.org/document/9636860/},
year = {2021}
}
```

If the Next Best View/Active Gaze control functionality of this package is used, please cite:
```
@article{FineanRAL,
   author = {Mark Nicholas Finean and Wolfgang Merkt and Ioannis Havoutis},
   doi = {10.1109/LRA.2021.3137545},
   issn = {2377-3766},
   issue = {2},
   journal = {IEEE Robotics and Automation Letters},
   month = {4},
   pages = {1095-1102},
   title = {Where Should I Look? Optimised Gaze Control for Whole-Body Collision Avoidance in Dynamic Environments},
   volume = {7},
   url = {https://ieeexplore.ieee.org/document/9661405/},
   year = {2022},
}
```