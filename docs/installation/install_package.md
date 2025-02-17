---
title: Package Installation
layout: default
permalink: /installation/rust
---

# Installing the Orbita3d control stack

The crate is written in Rust and is communicating difectly wit the EtherCAT IgH Master. It uses the rust wrapper crate ethercat-rs and builds on top of it to provide a more user friendly interface to the user.

## Prerequisites

In order to run this project, you need to have Rust installed on your machine. You can install it by following the instructions on the [official Rust website](https://www.rust-lang.org/tools/install).
Additionally, you will need to have the EtherCAT master installed on your machine. You can find the instructions on how to install it [here](installation_ethercat.md).

## Installation 

There are multiple ways of using this code in your projects:

- As a Rust library  - see the [Rust crate](../rust_crate)
- As a Python library - see the [Python library](../python)
- As a C library - see the [C bindings](../orbita_c)
- As a ROS2 node - see the [ROS node](../ros)