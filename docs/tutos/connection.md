---
title: Quick start
layout: default
---

# Connection guide

## Connecting the power
Connect the power the one of the pico-2 connectors (any one of the two available ones)

<img src="../../img/power.jpg" alt="Actuator" style="width: 400px;"/>

INFO:
- The actuator's supported power supply in in range of 12V to 24V. make sure to connect the power supply accordingly. 

WARNING:
The lower the voltage the lower the maximal achievable velocity of the actuator.

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
<img src="../../img/cable.jpg" alt="Actuator" style="width: 400px;"/>

And you should be ready to go. 

## Simple test

If you have the ethercat stack installed on your computer you can check if the actuator is connected by running the command `ethercat slaves` in the terminal. The command will show the list of connected slaves and the status of each one of them. 
If the actuator (ex. name: `RightWristOrbita3d`) is connected it should show something like this:

```shell
$ ethercat slaves

0  0:0  PREOP  +  RightWristOrbita3d
```

Then if you have the package installed from source, you can run a simple test to check if the actuator is working properly using the code examples `poulpe3d`:

```shell
RUST_LOG=info cargo run --release --example poulpe3d -- --start-server
```

Otehrwise you can use the python bindings to connect to the actuator and do the same test:

```python

import numpy as np
import time
from poulpe_ethercat_py import PyEthercatServer
from orbita3d import Orbita3dController

# Start the EtherCAT master
server = PyEthercatServer()
server.launch_server("path/to/ethercat_config.yaml")
# give some time for the master to start
time.sleep(1)


# connect to the actuator
orbita = Orbita3dController.from_config("path/to/orbita3d_config.yaml")

# enable the actuator
orbita.enable_torque()

# do a simple sinusoidal movement of the actuator
t0 = time.time()
while time.time() - t0 < 15:
    yaw = np.deg2rad(20) * np.sin(2 * np.pi * 0.15 * time.time())
    orbita.set_target_rpy_orientation((0.0, 0.0, yaw))
    time.sleep(0.001)

# disable the actuator
orbita.disable_torque()
```


<details markdown="1"><summary>Example ethercat config file</summary>

Example of a config file for the EtherCAT master (ex. `ethercat_config.yaml`):

```yaml
ethercat:
  master_id: 0
  cycle_time_us: 1000 # us - cycle time of the ethercat 1/frequency
  command_drop_time_us: 5000 # us (5ms default) 
  watchdog_timeout_ms: 500 # ms (500ms default)
  mailbox_wait_time_ms: 10000 #ms  (1s default)
```
</details>

<details markdown="1"><summary>Example orbita3d config file</summary>

Example of a config file for the Orbita3d actuator (ex. `orbita3d_config.yaml`):

```yaml
io: !PoulpeEthercat
  url: http://127.0.0.1:50098
  #name: NeckOrbita3d # it will use the name if available (optional but recomended)
  id: 0 # otherwise it will use the id (optional)
disks:
  #zeros: !ZeroStartup
  zeros: !FirmwareZero
  reduction: 5.333333333333333333
kinematics_model:
  alpha: 0.9424777960769379 # 54 degrees
  gamma_min: 0.6981317007977318 # 40 degrees
  offset: 0.0
  beta: 1.5707963267948966 # 90 degrees
  gamma_max: 3.141592653589793 # 180 degrees
  passiv_arms_direct: true
inverted_axes:
  - false
  - false
  - false
```
</details>


Both of the examples will do a short sinusoidal movement of the orbita actuator around the yaw axis.
<video width="400" controls>
    <source src="../../img/sinus.mp4" type="video/mp4">
    Your browser does not support the video tag.
</video>
