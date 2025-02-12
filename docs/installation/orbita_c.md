---
title: C-API library
layout: default
---

# Orbita3d C-API library

This repository contains the C-API library to control the Orbita3d actuator. It is generated using [cbindgen](https://github.com/mozilla/cbindgen).

This lets you control the actuator from any language that can call C functions. For example, you can use it from Python using the [orbita3d python bindings](../python).

The full API can be seen in the generated header file: [orbita3d.h]({{ config.repo_url }}/orbita3d_c_api/orbita3d.h).

## Build
* Prerequisites: Installed rust and cargo and the EtherCAT master.
* Install cbindgen: ```cargo install cbindgen```
* Build the library: ```cargo build --release``` (both static and dynamic)

The library is generated in the `target/release` folder.

NOTE:
You can find the full C-API documentation in the [API docs](../../api/orbita3d_c_api/).