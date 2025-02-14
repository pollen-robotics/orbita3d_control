---
title: Package Installation
layout: default
permalink: /installation/rust
---

# Installing the Orbita3d control rust package

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

## Simple test

If everithing went well you will be able to run the simple test, position your terminal in the `orbita3d_controller` folder and run:

```shell
RUST_LOG=info cargo run --release --example=poulpe3d -- --start-server
```

This test will do a short sinusoidal movement of the orbita actuator around the yaw axis.

<video width="300" controls>
    <source src="../../img/sinus.mp4" type="video/mp4">
    Your browser does not support the video tag.
</video>

NOTE:
This code will start the erhectcat master and try to connect to the slave with the id 0. If you want to change the id of the change it in the config file `config/ethercat_poulpe.yaml` - see [here]({{ config.repo_url }}/orbita3d_controller/config/ethercat_poulpe.yaml)

## Usage Rust API

The Rust API is composed of two main modules: `orbita3d_kinematics` and `orbita3d_controller`. The first one contains the forward and inverse kinematics of the actuator, while the second one contains the low-level communication and control of the actuator.

You can get inspired by the examples in the `examples` folder  to see the API in use - see [here]({{ config.repo_url }}/orbita3d_controller/examples/).

NOTE: 
You can also find the full Rust API documentation in the [API docs](../../api/orbita3d_controller/struct.Orbita3dController.html).


## Usage with Python and C

If you want to use the actuator with Python or C, you can use the bindings generated from the C-API library `orbita3d_c_api`. You can find the instructions on how to install the bindings for [python](../python) and [C](../orbita_c). And some more examples of usage in the [Python API example notebook]({{ config.repo_url }}/orbita3d_c_api/python/controller.ipynb) as well as in the [C-API examples]({{ config.repo_url }}/orbita3d_c_api/c-examples/).