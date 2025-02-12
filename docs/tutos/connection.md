---
title: Quick start
layout: default
---

# Connection guide

## Connecting the power
Connect the power the one of the pico-2 connectors (any one of the two available ones)

<img src="../../img/power.jpg" alt="Actuator" style="width: 400px;"/>

Once the power is connected, the actuator will start its initialisation procedure. Here is the video of it.
<video width="400" controls>
    <source src="../../img/init.mp4" type="video/mp4">
    Your browser does not support the video tag.
</video>

Once it is done it will show a green led light showing that it is ready to be used, otherwise it will show a red led light if there is an error.
See more info about the led lights [here](../led_blinking/).


## Connecting the ethercat cable
Then we can connect the ethercat cable (to any of the two connectors available).

<img src="../../img/ethercat.jpg" alt="Actuator" style="width: 400px;"/>

And you should be ready to go. 

## Simple test

If you have the ethercat stack installed in your computer you can check if the actuator is connected by running the command `ethercat slaves` in the terminal. The command will show the list of connected slaves and the status of each one of them. 
If the actuator (ex. name: `RightWristOrbita3d`) is connected it should show something like this:

```shell
$ ethercat slaves

0  0:0  PREOP  +  RightWristOrbita3d
```

You can run a simple test to check if the actuator is working properly using the code examples `poulpe3d`:

```shell
RUST_LOG=info cargo run --release --example poulpe3d -- --start-server
```
<video width="400" controls>
    <source src="../../img/sinus.mp4" type="video/mp4">
    Your browser does not support the video tag.
</video>