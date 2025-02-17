---
title: Rust crate
layout: default
---

# Orbita3d Rust crate

The crate is written in Rust and is communicating difectly wit the EtherCAT IgH Master. It uses the rust wrapper crate ethercat-rs and builds on top of it to provide a more user friendly interface to the user.

## Prerequisites

In order to run this project, you need to have Rust installed on your machine. You can install it by following the instructions on the [official Rust website](https://www.rust-lang.org/tools/install).
Additionally, you will need to have the EtherCAT master installed on your machine. You can find the instructions on how to install it [here](installation_ethercat.md).


## Importing the crate

The crate is designed to be used as a library in your project. You can include it in your `Cargo.toml` file as a dependency:

```toml
....
[dependencies]
...
orbita3d_control = { git = "https://github.com/pollen-robotics/orbita3d_control.git", branch = "develop" }
```

You can also include only sub-crates that you need. For example, if you only need the controller crate

```toml
....
[dependencies]
...
orbita3d_controller = { git = "https://github.com/pollen-robotics/orbita3d_control.git", branch = "develop" }
```

Then you can use it in your Rust code directly. See the examples in the `examples` folder for more information - [here]({{ config.repo_url }}/orbita3d_controller/examples/).


## Installing from source

If your applicaiton requires modifying the code and you want to build the project from source, you can do so by following these steps.
First clone the repository 

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