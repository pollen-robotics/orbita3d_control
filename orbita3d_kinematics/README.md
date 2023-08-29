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

## Maths

### Inverse kinematics

The method used is adapted from: "Kinematic analysis, optimization and programming of parallel robotic manipulator", Gosselin 1985

It is an analytical method that gives exact solutions in a closed form. However, due to the parallel mechanism characteristics, there are theoretically up to $2^3$ solutions (2 solutions for each passive arm, as solutions of a quadratic equation). In practice, only one solution is valid knowing the mounting orientation of the passive arms (either cw or ccw) and real world constraints, the method is able to find it.
The library also checks the range of angle of the solution ($\gamma_{min}$ and $\gamma_{max}$) which are the min and max angles allowed between the vertical arms.


### Forward kinematics

The forward kinematics of a parallel system is unfortunately often much more difficult to compute and usually no exact solution can be obtained. The method used here is a numerical method that solves a system of 8 equations obtained from geometric constraints of the system. The system is solved using the Levenberg-Marquardt algorithm.
As demonstrated in: "On the direct kinematics of a class of spherical three-degree-of freedom parallel manipulators", Gosselin, C. M., Sefrioui, J., & Richard, M. J. (1992). https://doi.org/10.1115/DETC1992-0193"
the system leads to a polynomial of degree 8, which leads to up to 8 solutions. Here again we filter out unfeasible solutions.


### Velocity

Forward and inverse velocity kinematics are computed using the Jacobian matrix: $J(\theta)\omega=\dot{\theta}$
The output Orbita platform velocity is expressed as a 3D velocity pseudo vector $\omega$ (the norm of the vector is the speed, the direction is the instantaneous axis of rotation).
The input motors velocity is expressed as a 3D vector $\dot{\theta}$.

### Torque

Forward and inverse static torque are computed using the Jacobian matrix: $J^T(\theta)F\=\tau$
The output Orbita platform torque $F$ is expressed as a 3D pseudo vector.
The input motors torque $\tau is expressed as a 3D vector.
