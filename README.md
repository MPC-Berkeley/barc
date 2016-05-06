# Berkeley Autonomous Race Car (BARC) Repo

The Berkeley Autonomous Race Car is a development platform for autonomous driving to achieve complex maneuvers such as drifting, lane changes, and obstacle avoidance. A 1/10 scale RC car and an embedded Linux computer make up the hardware platform of the project. This project aims to be fully open-source. The data collection process is cloud-based and brings new dimensions to the Vehicle Dynamics and Control Theory teaching and research world.

This site is home to the repository. The main folders in this repo include

* CAD
  * STL and DWX files for fabricating the aluminum deck and side brackets from for the vehicle, and the ABS cover for the odroid
* Dator
  * Web server for cloud robotics. Provides a standard way to record data and events from one or more local computers for later analysis. Based on [this repo](https://github.com/bwootton/Dator) from Bruce Wooton
* Arduino
  * Files to program the arduino to (1) send commands to the electronic speed control (ESC) unit and the servo, and to (2) acquire measurements from the encoders and ultrasound sensors</span></li>
* Scripts
  * Bash programs that set up environment variables and launch the local server upon boot
* Utility
  * Miscellaneous useful scripts
* Workspace
  * Robotic Operating System (ROS) workspace that contains the barc package. This package holds the source code to control the vehicle using the ROS framework

## Getting started
### Rebuild ROS and reflash arduino
After git cloning or pulling, rebuild the ROS workspace by opening a terminal and executing the following (alias) command
(Make sure your arduino is plugged in, and on port /dev/ttyUSB0)

```rebuild_system ```

To reflash your arudino, you can run the following command

`nanorst`

### Register with the cloud
In your home directory, edit a filnamed `team_name.sh` to define a username, then in four separate terminals, execute the following
```
source ~/barc/scripts/reset_database.sh 
roscore
rosrun data_service service.py
source ~/barc/scripts/register_cloud.sh
```

### Running NoMachine

To change the resolution:

```xrandr --fb <width>x<height>```
