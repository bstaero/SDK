This is the public SDK for the Black Swift Technologies SwiftCore flight
management system. More information on the products this works with can be
found on our website https://bst.aero

For information on using this with a gazebo multi-rotor simulation, see here:
https://gitlab.com/bstaero/sdk/-/wikis/gazebo-setup

For information interfacing with the BST SwiftFlow wind probe, see here:
https://gitlab.com/bstaero/sdk/-/wikis/swiftflow-interface

## Python SDK

### Prerequisites

Installing the Python SDK requires the following to be installed on your machine:

- swig
- python3-dev

### Install

`pip install BSTPythonSDK`

### Import

`import bst_python_sdk`

### Usage

#### Parse Log

```py
from bst_python_sdk.logparse import Parser

log_path = "path/to/log.bin"
parser = Parser()

# parsed_log will be a dict containing all packets from the provided log
parsed_log = parser.parse_log(log_path)
```

#### Log -> NetCDF

```py
from bst_python_sdk.log_to_nc import convert_to_nc

log_path = "path/to/log.bin"

# output will be a list of the converted logs in netcdf format
# Ex: ["log_010_FW0001.nc", "log_010_SwiftStation.nc"]
output = convert_to_nc(log_path)
```
