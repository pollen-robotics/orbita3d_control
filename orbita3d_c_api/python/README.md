# Orbita3d python bindings

This folder contains the Python bindings generated using the [orbita3d C-API library](../README.md). It is generated using [maturin and cffi](https://github.com/PyO3/maturin).

## Installation

- From the `orbita3d_c_api` directory, build the C-API library by running `cargo build --release`
- Create a virtual environment and activate it `python -m venv venv && source venv/bin/activate`
- Install maturin in your Python virtual environment. For example, using pip: `pip install maturin`
- then `maturin develop` to build and install the Python bindings in the virtual environment.

Then you should be able to directly import the orbita3d module in your Python code:

```python
import orbita3d
```

## Usage

```python

import numpy as np
import time

from orbita3d import Orbita3dController

orbita = Orbita3dController.from_config("path_to_my_config_file")

orbita.enable_torque(True)

t0 = time.time()
while time.time() - t0 < 15:
    ring = np.deg2rad(20) * np.sin(2 * np.pi * 0.15 * time.time())
    orbita.set_target_orientation((ring, 0.0))

    time.sleep(0.001)

orbita.disable_torque()
```

_You can find examples of config files in the [config](../../orbita3d_controller/config/) folder._

See [Python API example notebook](./API_example.ipynb) for more complex usage.%
