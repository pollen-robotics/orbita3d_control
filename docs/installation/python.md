---
title: Python bindings
layout: default
---

# Orbita3d Python bindings

The Orbita3d control stack is written in Rust and provides C-API library as well as Python bindings.
The python bindings are generated using [maturin and cffi](https://github.com/PyO3/maturin).

In order to use the python bindings you will need to install on your machine:

- Rust - [installation guide](https://www.rust-lang.org/tools/install)
- EtherCAT IgH Master (for the EtherCAT communication) - [installation guide](installation_ethercat) 

Then there are two ways to install the python bindings:

- Using pip directly from the repository
- Building the bindings from the source


## Python-only installation with pip

If you are using the python-only version of the code you can install the code directly from the repository. 

```shell
pip install git+https://github.com/pollen-robotics/orbita3d_control.git#subdirectory=orbita3d_c_api --verbose
```

NOTE: <details><summary>Installing specific versions or branches of the code</summary>
> 
> You can also specify the version of the code that you want to install by using the tag. For example, if you want to install the `v1.1.4` version of the  code you can use the following command (or any other tag/branch that you want to install):
> 
> ```shell
> pip install git+https://github.com/pollen-robotics/orbita3d_control.git@v1.1.4#subdirectory=orbita3d_c_api --verbose
> ```
> 
> </details>

Once this is installed you will be able to import the module in your python code:

```python
import orbita3d
```


## Build python bindings from source

First clone the directiory and build the C-API library:

```shell
git clone https://github.com/pollen-robotics/orbita3d_control.git
```

Then build the C-API library and the python bindings:

```shell
cd orbita3d_control/orbita3d_c_api
cargo build --release
```

Create a virtual environment (using `venv` or `anaconda`) and activate it and install maturin:

```shell
python -m venv venv
source venv/bin/activate
pip install maturin
```

INFO:
If necessary install the vnenv with `sudo apt-get install python3-venv`


Then build and install the Python bindings in the virtual environment:

```shell
maturin develop
```

Then you should be able to directly import the orbita3d module in your Python code:

```python
import orbita3d
```



## Install poulpe_ethercat_controller (optional)

We also suggest you to install the `poulpe_ethercat_controller` python binding as well (regardless if installing orbita3d with pip or from source). THis package will allow you to start the EtherCAT master from python. 

You can install it by running:

```shell
pip install git+https://github.com/pollen-robotics/poulpe_ethercat_controller.git#subdirectory=poulpe_ethercat_py --verbose
```

NOTE: <details><summary>`poulpe_ethercat_controller`, `firmware_Poulpe` and `orbita3d_control` compatibility</summary>
>  
> The rule of thumb is that the `poulpe_ethercat_controller` at its default branch will be compatible with the latest version of the `firmware_Poulpe` as well as the latest version of `orbti3d_controller`.
>
> You can find more info in the [installation guide](https://pollen-robotics.github.io/poulpe_ethercat_controller/installation/) on how to install the `poulpe_ethercat_controller` and the `firmware_Poulpe` version that you are using. 
>
> But in general the compatibility between the `poulpe_ethercat_controller` and the `firmware_Poulpe` is as follows: 
>
> `firmware_poulpe` version | `poulpe_etehract_controller` version
> --- | ---
> v0.9.x | 0.9.x
> v1.0.x | 1.0.x or higher
> v1.5.x | 1.5.x 
> 
> And you can install a specific tag/branch of the code by specifying it in the installation command. For example, if you want to install the `1.5.4` version of the code you can use the following command:
>
> ```shell
>pip install git+https://github.com/pollen-robotics/poulpe_ethercat_controller.git@1.5.4#subdirectory=poulpe_ethercat_py --verbose
> ```
> 
> </details>

Once you have both installed you can start the EtherCAT master and the actuator from python:

```python
from poulpe_ethercat_py import PyEthercatServer
import time

# Start the EtherCAT master
server = PyEthercatServer()
server.launch_server(".../path/to/ethercat/config.yaml")
# give some time for the master to start
time.sleep(1)

import orbita3d
# Start the actuator
controller = orbita3d.Orbita3dController(".../path/to/orbita3d/config.yaml")

....
# do something useful :D
```

## Simple example applicaiton

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
orbita.enable_torque(reset_target=True)

# do a simple sinusoidal movement of the actuator
t0 = time.time()
while time.time() - t0 < 15:
    yaw = np.deg2rad(20) * np.sin(2 * np.pi * 0.15 * time.time())
    orbita.set_target_rpy_orientation((0.0, 0.0, yaw))
    time.sleep(0.001)

# disable the actuator
orbita.disable_torque()
```

You can find examples of config files in the [config]({{ config.repo_url }}/orbita3d_controller/config/) folder.



NOTE:
> You can find the full API in the [API docs](../../api/orbita3d_c_api/python/). <br>
> - See [Python API example notebook]({{ config.repo_url }}/orbita3d_c_api/python/controller.ipynb) for more complex usage.


### If installing from source 

You can also start the server using the provided RUST example in the `orbita3d_controller` crate:

```shell
RUST_LOG=info cargo run --release --example=ethercat_server
```

And then you dont need to install the `poulpe_ethercat_controller` directly and your python applicaiton will look like this:

```python
from orbita3d import Orbita3dController

# connect to the actuator
orbita = Orbita3dController.from_config("path/to/orbita3d_config.yaml")

# enable the actuator
orbita.enable_torque(reset_target=True)

# do a simple sinusoidal movement of the actuator
t0 = time.time()
while time.time() - t0 < 15:
    yaw = np.deg2rad(20) * np.sin(2 * np.pi * 0.15 * time.time())
    orbita.set_target_rpy_orientation((0.0, 0.0, yaw))
    time.sleep(0.001)

# disable the actuator
orbita.disable_torque()
```


