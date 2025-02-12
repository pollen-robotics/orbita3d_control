---
title: ROS integration
layout: default
---

# ROS2 integration

* To integrate into a ROS2 workspace, simply clone this repository into the src/ directory of your workspace.
* Make sure to have the [ros2_pollen_toolbox](https://github.com/pollen-robotics/ros2_pollen_toolbox) in the same workspace
* Make sure to have [ros2_rust]() setup.
   <details markdown="1">
    <summary>If its your first time using rust with ros</summary>
   Make sure to install:
    
   ```
   pip install git+https://github.com/colcon/colcon-cargo.git
   pip install git+https://github.com/colcon/colcon-ros-cargo.git
   ```
   
   </details>
* Make sure to use "patched version" of cargo-ament-built (see [this PR](https://github.com/ros2-rust/cargo-ament-build/pull/3) for more details)
```
cargo install --debug --git https://github.com/jerry73204/cargo-ament-build.git  --branch conditionally-copy-cargo-lock-file
```
* Update colcon-cargo with: `python3 -m pip install --upgrade --force-reinstall git+https://github.com/pollen-robotics/colcon-cargo.git`

## Build the hwi

* `colcon build --symlink-install --packages-up-to orbita3d_system_hwi`

## Build the description

* `colcon build --symlink-install --packages-up-to orbita3d_description`

* See the [orbita3d_description](orbita3d_description/README.md) package for more details.