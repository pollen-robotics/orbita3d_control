---
title: Home
layout: default
nav_order: 1
---
# Orbita3d control stack


[![Rust Lint and Test](https://github.com/pollen-robotics/orbita3d_control/actions/workflows/rust.yml/badge.svg)](https://github.com/pollen-robotics/orbita3d_control/actions/workflows/rust.yml)
[![Python checks](https://github.com/pollen-robotics/orbita3d_control/actions/workflows/python.yml/badge.svg)](https://github.com/pollen-robotics/orbita3d_control/actions/workflows/python.yml)


## Overview

This repository contains all libraries required to control an Orbita3d actuator. It allows:

* to fully control the actuator using Python
* access the full API (Rust package or C-API library)
* integrate the actuator in your ROS robot (URDF and ROS2 humble control hardware interface)

## Usage

In order to run this project, you need to have Rust installed on your machine. You can install it by following the instructions on the [official Rust website](https://www.rust-lang.org/tools/install).
Additionally, you will need to have the EtherCAT master installed on your machine. You can find the instructions on how to install it [here](installation_ethercat.md).


## Installation

To install the project, you can clone the repository 

```shell
git clone git@github.com:pollen-robotics/orbita3d_control.git
```

Then, you can build the project by running:

```shell
cargo build --release
```

### Via Rust

See [orbita3d_control](installation/install_package) for more details.

### Via Python

See [orbita3d python bindings](installation/python) for more details.


### Via C

See [orbita3d C-API](installation/orbita_c) for more details.

### ROS2 integration

#### Setup

See [ROS2 integration](installation/ros) for more details.

## Contents

This repository contains the following sub-packages:

### Rust packages for control

* [orbita3d_kinematics]({{config.repo_url}}/orbita3d_kinematics/README.md): forward/inverse kinematics model (in Rust)
* [orbita3d_controller]({{config.repo_url}}/orbita3d_controller/README.md): low-level communication (serial or ethercat) and control (in Rust)

### C-API library and Python bindings

* [orbita3d c_api]({{config.repo_url}}/orbita3d_c_api/README.md): plain C-API library to control the actuator
* [orbita3d python bindings]({{config.repo_url}}/orbita3d_c_api/python/README.md): Python bindings for the C-API library

### ROS2 humble integration

* [orbita3d_description]({{config.repo_url}}/orbita3d_description/README.md): URDF/ros2_control description of the actuator
* [orbita3d_system_hwi]({{config.repo_url}}/orbita3d_system_hwi/README.md): ros2 control hardware system interface for the actuator

## Changelog

See [changelog](https://github.com/pollen-robotics/orbita3d_control/releases).

## Related repositories

### Flipsky design

* [Mechanical design](https://cad.onshape.com/documents/d108475d689e47a5561e996c/w/1d6ae1891c12354f3ac85124/e/f54fa638e232e8705b683608)
* [Electronics design](https://github.com/pollen-robotics/orbita3d_elec)
* [Firmware](https://github.com/pollen-robotics/firmware_Orbita3dofs/tree/Flipsky_FSESC6.7-pro)