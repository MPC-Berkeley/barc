# sweep-ros

## Dependencies
The Sweep ROS Driver and Node requires that the libsweep library from the [sweep-sdk](https://github.com/scanse/sweep-sdk) be installed on the computer.

See the [libsweep README](https://github.com/scanse/sweep-sdk/tree/master/libsweep) for full installation instructions, or follow the brief guide below:

```bash
# clone the sweep-sdk repository
git clone https://github.com/scanse/sweep-sdk

# enter the libsweep directory
cd sweep-sdk/libsweep

# create and enter a build directory
mkdir -p build
cd build

# build and install the libsweep library
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig
```

## Scanse Sweep ROS Driver and Node

This node is currently publishing a `pointcloud2` msg. This is because the Sweep device does not use fixed azimuth intervals. For more details see the Sweep [theory of operation](https://support.scanse.io/hc/en-us/articles/115006333327-Theory-of-Operation). Use `sweep2scan.launch` for conversion to `laserscan` msg.

## Firmware Compatibility:
Currently, sweep-ros is only compatible with sweep `firmware v1.1` or greater.

You can check the firmware version installed on your sweep device by using a serial terminal (see [manual](https://s3.amazonaws.com/scanse/Sweep_user_manual.pdf)) or more easily using the sweep visualizer (see [instructions](https://support.scanse.io/hc/en-us/articles/224557908-Upgrading-Firmware)).