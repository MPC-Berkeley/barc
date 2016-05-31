# Berkeley Autonomous Race Car (barc) Repo

The Berkeley Autonomous Race Car is a development platform for autonomous driving to achieve complex maneuvers such as drifting, lane changes, and obstacle avoidance. A 1/10 scale RC car and an embedded Linux computer make up the hardware platform of the project. This project aims to be fully open-source. The data collection process is cloud-based and brings new dimensions to the Vehicle Dynamics and Control Theory teaching and research world.

This site is home to the repository. The main site for the project is [here](http://www.barc-project.com/). 

The primary folders in this repository include

* CAD
  * DWX files for fabricating the deck and side brackets for the chassis, and STL files for fabricating the sensor mounts (e.g. hall effect sensor, camera, ultrasound) and the cover for the odroid.
* Dator
  * Web server for cloud robotics. Provides a standard way to record data and events from one or more local computers for later analysis. Based on [this repo](https://github.com/bwootton/Dator) from Bruce Wooton
* Arduino
  * Files to program the arduino to (1) send commands to the electronic speed control (ESC) unit and the servo, and to (2) acquire measurements from the encoders and ultrasound sensors</span></li>
* Scripts
  * Bash programs that set up environment variables and launch the local server upon boot
* MATLAB
  * Useful MATLAB scripts for processing ROS bag file. The bag files store all the message data (time stamped sensor measurements, actuator commands, etc) during an experiment
* Workspace
  * **Robotic Operating System (ROS)** workspace that contains the barc package. This package holds the source code to control the vehicle using the ROS framework

All software to control the vehicle resides in the *Arduino* and *Workspace* folders.

## Getting started
### Rebuild ROS and reflash arduino
After git cloning or pulling, rebuild the ROS workspace by opening a terminal and executing the following (alias) command
(Make sure your arduino is plugged in, and on port /dev/ttyUSB0)

`rebuild_system`

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

### Useful linux commands / tools
Note that for typing commands, like the following, linux supports [tab completion](http://www.howtogeek.com/195207/use-tab-completion-to-type-commands-faster-on-any-operating-system/), which can dramatically help speed up typing commands.

+ Change the resolution of the screen. (useful when using NoMachine to connect remotely)
 + `xrandr --fb <width>x<height>`
+ Print the hostname settings (useful for displaying information on operating system, kernel, and architecture)
 + `hostnamectl`
+ Print detailed information about the connected usb device
 + `lsusb` (less info) or `usb-devices` (more info)
+ Switch easily between several programs within one terminal. Watch [this video](https://www.youtube.com/watch?v=BHhA_ZKjyxo) for a tutorial on how to use tmux
 + `tmux`
+ Search code base for a text string, read [here](http://conqueringthecommandline.com/book/ack_ag) for in-depth examples 
 + `ag <text_string>`

### Useful ROS commands
Below is a list of some commonly used commands. Check [this link](http://wiki.ros.org/ROS/CommandLineTools) for a comprehensive list
+ Change to a ROS package directory
 + `roscd <package_name>`, for example `roscd barc`
+ Run a single ROS node
 + `roscore` (in one terminal)
 + `rosrun <package> <file_name>` (in another terminal) 
+ Run a launch file
 + `roslaunch <package> <file_name>.launch` 
+ Display the active topics
 + `rostopic list`
+ Print the messages coming from a topic
 + `rostopic echo <topic>`
+ Print the publishing rate of a topic
 + `rostopic hz <topic>`

### Useful resources

Recommended reading and resources
+ [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/), by Jason M. O'Kane, and this [basic tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) for writing ROS nodes in python
+ [Julia JuMP] (https://jump.readthedocs.io/en/latest/) 
+ [Git - the simple guide] (http://rogerdudler.github.io/git-guide/), or for a more in-depth reading, consider [this](https://www.atlassian.com/git/tutorials/ ) tutorial by Atlassian
