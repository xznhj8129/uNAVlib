# uNAVlib
**[Version: 0.1.1](CHANGELOG)**\
\
uNAV as in Unmanned INAV, or "u do the nav, i'm busy".\
Fork of the now-inactive [YAMSPy](https://github.com/thecognifly/YAMSPy) library (Yet Another Implementation of [Multiwii](https://github.com/multiwii) Serial Protocol Python Interface), created by Ricardo de Azambuja; (specifically the [yb/stablle branch](https://github.com/thecognifly/YAMSPy/tree/yb/stable)). It is an SDK meant for developing the autonomous capabilities of [INAV](https://github.com/INAVFlight/INAV) and the potential for use with a companion computer (CC) such as a Raspberry Pi, NVIDIA Jetson, etc; basically allowing MAVSDK/Dronekit-like development for INAV. This assumes you already know what you are doing with INAV and know how to flash firmware to your board, set up UART connections, build custom firmware, flight mode configuration, etc. See old_readme.md for original information on installation, usage, advanced configuration, etc. 

## Why?
I happened to try INAV first and liked it, Ardupilot is way more capable, but also far more complex, and does not support as many boards as INAV, which is a fork of CleanFlight and a cousin of Betaflight. There was absolutely no SDK similar to pymavlink or mavsdk for MSP, so i had to do it myself. Developing INAV's autonomous capabilities and MSP's functionality so it compares more to MAVLink would open up a lot of interesting options for those who use it. I forked and renamed YAMSPy because i'm probably going to completely change it. Betaflight will not be supported.

## Disclaimer ## 
This library is in active development and may change quickly, with no guarantees to back-compatibility or proper version control. I am a complete beginner at flight control software. AI will be used to develop it in parts when convenient. Caveat emptor. Use at your own and other's risk. This experimental work is provided to you as-is, with no guarantees whatsoever. Unexpected, spurious, undefined and unexplicable actions, failure states and malfunctions must be expected at any time for any reason, and it's safe, legal and responsible use to control drones/UAVs is the responsibility of the user. None of the authors, contributors, supervisors administrators, employers, friends, family, vandals, or anyone else connected (or not) with this project, in any way whatsoever, can be made responsible for the use of the information (code) contained or linked from here. Matthew 27:24.

## New Features ##
* Asyncio-based flight controller class object with simplified functions for easy mission scripting
* YAMSPy monster \__init__ class refactored and split into modules
* enums extracted from INAV source into dictionary imports for mapping and simpler programming
* geographic functions for navigation

## Planned features:
* Mission Control object using asyncio
* PID-based flight control
* Automated tests
* Implement/reproduce [MWPtools](https://github.com/stronnag/mwptools) functionality and tools
* Mavlink compatibility if possible (direct INAV source contribution might be much better)
* Failure mode tests and failsafes
* C++ implementation

## Utilities
* tools.generate_msp_override_bitmask: Generates the correct channel bitmask for MSP override.
* tools.gen_mode_config: Connects to your board and generates json file containing basic board info and mappings of your aux modes configuration for easier scripting.
* tools.generate_inav_enums: reads INAV source files to generate enums/inav_enums.py
* tools.generate_msp_codes: reads INAV source files to generate enums/msp_codes.py

## TODO:
See [TODO](/TODO)

## DONE:
* First example of asyncio-based mission control
* Refactor __init__.py into more manageable structure
* Import all INAV enums into dictionaries
* Generate enums automatically from INAV source
* Basic missions proof of concept

## Examples
[Examples](/examples) (more examples will come as main codebase improves, old YAMSPy examples may break)

## Software Requirements:
* Python >= 3.10
* INAV >= 7.1
* asyncio >= 3.4.3
* simple-pid
* geographiclib
* mgrs
* geojson

## MSP Override
See https://codeberg.org/stronnag/msp_override \
Allows MSP protocol to override RC channels coming from a transmitter, allowing use of both an RC transmitter and MSP control via CC.
* Firmware must be built with  `USE_MSP_RC_OVERRIDE` (un-comment in `src/main/target/common.h` or append to `src/main/target/TARGET_NAME/target.h`).
* The override mask `msp_override_channels` is set for the channels to be overridden by entering `set msp_override_channels =  (your bitmask here` in the CLI. (see unavlib/tools/generate_msp_override_bitmask.py to generate the correct one)
* Flight mode `MSP RC Override` is active.

## Setting up your flight controller (FC):
This project starts with INAV 7.1 as basis. 
1. Build the firmware with MSP_RC_OVERRIDE and flash
2. Follow standard INAV setup procedures.
3. Set up MSP telemtry UART port to connect to.
4. Connect telemetry port to your Pi's UART or to a USB-TTL converter (ex: CP2102)
5. Test connection by connecting to FC with Inav configurator using that serial connection (ex: /dev/ttyUSB0)
6. 

## Troubleshooting
If you can't connect (talk) to the FC:
1. Check if you enabled MSP in the correct UART using INAV-configurator
2. Make sure you connected the cables correctly: TX => RX and RX => TX
3. Verify the devices available using ```ls -lh /dev/serial*``` (or dev/ttyUSB* / dev/ttyACM*)
4. Verify your wireless connection if using telemetry radios or similar (i haven't tried')
5. Turn it off and on again
6. Appease the machine-spirit

## Installation:
Option #1: Clone the repo so you will have the examples
```
$ git clone https://github.com/xznhj8129/uNAVlib.git
$ cd unavlib
$ sudo pip3 install .
```

Option #2: Install directly from git (the `--upgrade` is to make sure it will install the last commit, even if the version number didn't increase)
```
$ sudo pip3 install git+https://github.com/xznhj8129/uNAVlib --upgrade

```
or to select a branch (e.g. [proxy](https://github.com/xznhj8129/uNAVlib/tree/proxy)):

```
$ pip install git+https://github.com/xznhj8129/uNAVlib@proxy --upgrade
```

On Linux you may need to add your user to the dialout group:
```
$ sudo usermod -a -G dialout $USER
```

## Proxy
This fork's starting point is the [yb/stable branch](https://github.com/thecognifly/YAMSPy/tree/yb/stable) of YAMSPy that has a new, experimental, script that allows you to play with MSP messages like you would do with [MAVProxy](https://ardupilot.org/mavproxy/). This proxy will allow many scripts to share the same UART connected to the FC. Then, you can use unavlib in TCP mode (`use_tcp=True`) to connect to the FC using one of the ports created by the proxy (only one connection per port since it's TCP). To launch the proxy creating the ports `54310`, `54320`, and `54330`:

```
$ python -m unavlib.msp_proxy --serial /dev/ttyACM0 --ports 54310 54320 54330
```
In your script using unavlib you need to set `use_tcp=True` and pass the port number as the device. Check the script [`simpleUI_tcp.py`](/examples/simpleUI_tcp.py) in the `examples` folder.

## Contributions:
All are welcome and encouraged to contribute!

## Acknowledgments:
[Ricardo de Azambuja](https://github.com/ricardodeazambuja), [Tom](https://github.com/cmftom), [Yann](https://github.com/yannbouteiller), the [Cognifly Project](https://github.com/thecognifly/) and all other contributors for developing the original YAMSPy library.\
[Jonathan Hudson](https://github.com/stronnag) for his work on INAV and the examples on how to use the little-documented autonomous control features.
