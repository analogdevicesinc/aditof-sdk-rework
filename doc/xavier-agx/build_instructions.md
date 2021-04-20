# Xavier AGX Build Instructions


## Building the SDK only

### SDK dependencies
To build the SDK and run the included applications and example code the following dependencies must be installed in the system:
 - v4l-utils
 - libopencv-dev
 - cmake
 - glog v0.3.5
 - libwebsockets v3.1
 - protocol buffers v3.9.0

The SD card image already contains all the SDK dependencies and there's no need to install them again. To update and build the SDK just follow the steps below.

```console
analog@xavier:~/workspace/aditof-sdk-rework$ git pull
analog@xavier:~/workspace/aditof-sdk-rework$ cd build
analog@xavier:~/workspace/aditof-sdk-rework/build$ cmake -DXAVIER=1 ..
analog@xavier:~/workspace/aditof-sdk-rework/build$ make -j4
```

## Linux Kernel
The SD card image already contains the customized Linux kernel image and required devicetree to support the ToF Camera connected to CSI connector on Xavier.
If rebuilding the kernel or devicetree is needed please follow the [instructions](https://wiki.analog.com/resources/eval/user-guides/ad-96tof1-ebz/ug_xavier_agx).
