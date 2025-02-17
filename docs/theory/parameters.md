---
title: Model parameters
layout: default
---

## Orbita Model parameters

The 3D orientation of the Orbita platform is defined by a rotation matrix representing the rotation of the frame `(X_orbita, Y_orbita, Z_orbita)` relative to the world frame `(X_world, Y_world, Z_world)`. In the initial position, the two frames are co-axial.

The angles `(theta_0, theta_1, theta_2)` represent the angular positions of the 3 disks that move the mechanism.

<img src="../../img/top_view.jpg" width="700">

Due to its construction, Orbita can perform infinite rotations around the Z-axis (yaw). However, the amplitude of movements on the other two axes is limited by the mechanism, which can reach singularities or be blocked by collisions of mechanical parts.

The possible amplitudes on the X and Y axes are determined by 4 parameters.

Two mathematical parameters, the angles alpha and beta, fully describe the theoretical kinematics of Orbita's parallel mechanism.

<img src="../../img/alpha_beta.jpg" width="700">

The choice of the two angles alpha and beta directly changes the possible inclination angles of the Orbita platform in such a way that:

- The larger the alpha, the greater the possible inclinations of the platform
- The smaller the alpha, the greater the output torque
- The smaller the alpha, the better the precision in output
- Beta = 90° allows the greatest deflection; larger or smaller, the inclination space decreases

Two additional parameters, the angles gamma_min and gamma_max, are related to mechanical design. They define the minimum and maximum angular distances that separate the active arms, notably to avoid collisions and singularity points.

<img src="../../img/gamma.jpg" width="700">

These two angles reduce the theoretical amplitude of the platform and allow for the calculation of the actual amplitude.


### Parameters in the code

This package manages the model parameters through the config files, that can be found in the `config` folder of the `orbita3d_controller` package.

Here are the parameters that can be set in the config file:
```yaml
...
kinematics_model:
  alpha: 0.9424777960769379 # 54 degrees
  gamma_min: 0.6981317007977318 # 40 degrees
  offset: 0.0
  beta: 1.5707963267948966 # 90 degrees
  gamma_max: 3.05433 # 175 degrees
...
```

