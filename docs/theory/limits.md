---
title: Movemnt limits
layout: default
---

## Orbita Model

The 3D orientation of the Orbita platform is defined by a rotation matrix representing the rotation of the frame `(X_orbita, Y_orbita, Z_orbita)` relative to the world frame `(X_world, Y_world, Z_world)`. In the initial position, the two frames are co-axial.

The angles `(theta_0, theta_1, theta_2)` represent the angular positions of the 3 disks that move the mechanism.

![axis](../../img/top_view.jpg)

Due to its construction, Orbita can perform infinite rotations around the Z-axis (yaw). However, the amplitude of movements on the other two axes is limited by the mechanism, which can reach singularities or be blocked by collisions of mechanical parts.

The possible amplitudes on the X and Y axes are determined by 4 parameters.

Two mathematical parameters, the angles alpha and beta, fully describe the theoretical kinematics of Orbita's parallel mechanism.

![Alpha beta angle definition](../../img/alpha_beta.jpg)

The choice of the two angles alpha and beta directly changes the possible inclination angles of the Orbita platform in such a way that:

- The larger the alpha, the greater the possible inclinations of the platform
- The smaller the alpha, the greater the output torque
- The smaller the alpha, the better the precision in output
- Beta = 90° allows the greatest deflection; larger or smaller, the inclination space decreases

Two additional parameters, the angles gamma_min and gamma_max, are related to mechanical design. They define the minimum and maximum angular distances that separate the active arms, notably to avoid collisions and singularity points.

![Gamma angle definition](../../img/gamma.jpg)

These two angles reduce the theoretical amplitude of the platform and allow for the calculation of the actual amplitude.
