# Analog Devices 3D ToF software suite 

## This repository is work in progress. It is not usable at the moment. Don't use it!
## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADSD3100 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera, IR stream and depth data. Windows support are built into the SDK as well as sample code and wrappers for various languages including Python, C/C++ and Matlab.

License : [![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/analogdevicesinc/aditof-sdk-rework/blob/master/LICENSE)
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)]()

## Supported host platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below:.

| Operating System | Documentation | GitHub master status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Windows | [User Guide](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_windows) <br> [Build Instructions](doc/windows/build_instructions.md) | [![Build status](https://ci.appveyor.com/api/projects/status/46t36hmy77ejrf88/branch/master?svg=true)](https://ci.appveyor.com/project/analogdevicesinc/aditof-sdk-rework/branch/master) | [![aditof-demo installer](https://img.shields.io/badge/release-aditof_demo_installer-blue.svg)](https://github.com/analogdevicesinc/aditof-sdk-rework/releases/latest) |

## Supported embedded platforms

For more details on building the SDK on an embedded platform please check the **User Guide** specified below:
### ITOF camera
| Operating system | Documentation |
| --------- | ----------- |
| Windows | [Build instructions](doc/itof/windows_build_instructions.md) |
| NXP | [Build instructions](doc/itof/nxp_build_instructions.md) |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## SDK documentation

From an architectural point of view, the SDK consists of two layers. One layer is the high level API that allows clients to easily grab a camera object, configure it and request frames. The other layer is the low level API which exposes the interface through which low level operations can be made to the camera hardware.

For more details about the SDK check the links below:

[Software stack documentation](https://github.com/analogdevicesinc/aditof-sdk-rework/blob/master/sdk/readme.md)

[API Doxygen documentation](https://analogdevicesinc.github.io/aditof-sdk-rework/)

[Building and installing the SDK](https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/cmake/)

## SDK examples
| Example | Language | Description |
| --------- | ------------- | ----------- |
| tof-viewer | <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/examples/tof-viewer"> C++ </a> | Add description here |
| data-collect | <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/examples/data-collect"> C++ </a> | Add description here |
| first-frame | <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/examples/first-frame"> C++ </a> <br> <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/bindings/python/examples/first_frame"> Python </a> | An example code that shows the steps required to get to the point where camera frames can be captured. |
| first-frame-network | <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/examples/first-frame-network"> C++ </a> | An application that shows how to use the aditof sdk to talk to a remote ToF camera over the network. |
| device_example | <a href="https://github.com/analogdevicesinc/aditof-sdk-rework/tree/master/bindings/python/examples/device_example"> Python</a> | A simple example of how to get access to the low-level API of the camera. |

## Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Applications specific to various targets and hosts |
| bindings | SDK bindings to other languages |
| ci | Useful scripts for continuous integration |
| doc | Documentation |
| examples | Example code for the supported programming languages |
| scripts | Useful development scripts |
| sdk | SDK source code |
