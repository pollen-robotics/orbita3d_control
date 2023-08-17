# Orbita 3D kinematics model

## Overview

Implements the kinematics model for the Orbita 3D project, forward and inverse computation for:
* [x] position
* [x] velocity
* [x] torque

The crate can be directly used in Rust. It also provides a [C-API](../orbita3d_c_api/) and a [Python binding](../orbita3d_c_api/python/).

## Model definition

- $thetas$: disk position angles ($theta_0$ corresponds to the X axis, then $theta_1$ and $theta_2$)
- $rot$: 3D rotation matrix of the effector

All values are expressed in radians.

For details on the model, see the [this repository](https://github.com/pollen-robotics/orbita-model).