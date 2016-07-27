# Berkeley Autonomous Race Car (barc) Repo

The Berkeley Autonomous Race Car is a development platform for autonomous driving to achieve complex maneuvers such as drifting, lane changes, and obstacle avoidance. A 1/10 scale RC car and an embedded Linux computer make up the hardware platform of the project. This project aims to be fully open-source. The data collection process is cloud-based and brings new dimensions to the Vehicle Dynamics and Control Theory teaching and research world.

This site is home to the repository. The main site for the project is [here](http://www.barc-project.com/).

The primary folders in this repository include

* docs
  * Overview about the mechanical, electrical, and software deign of the vehicle. Descriptions of the vehicle models used for some control algorithms
* CAD
  * DWX files for fabricating the deck and side brackets for the chassis, and STL files for fabricating the sensor mounts (e.g. hall effect sensor, camera, ultrasound) and the cover for the odroid.
* Dator
  * Web server for cloud robotics. Provides a standard way to record data and events from one or more local computers for later analysis. Based on [this repo](https://github.com/bwootton/Dator) from Bruce Wooton
* arduino
  * Files to program the arduino to (1) send commands to the electronic speed control (ESC) unit and the servo, and to (2) acquire measurements from the encoders and ultrasound sensors</span></li>
* scripts
  * Bash programs that set up environment variables and launch the local server upon boot
* MATLAB
  * Useful MATLAB scripts for processing ROS bag file. The bag files store all the message data (time stamped sensor measurements, actuator commands, etc) during an experiment
* workspace
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
In your home directory, edit a file named `team_name.sh` to define a user name, then in four separate terminals, execute the following
```
source ~/barc/scripts/reset_database.sh
roscore
rosrun data_service service.py
source ~/barc/scripts/register_cloud.sh
```
To run an experiment, enter the following three commands in three separate terminals. This should launch a single gui window with four different pluggins. (Note, experiments launched via the command line using `roslaunch barc <file_name>.launch` may save data to bagfiles, but the data will not be pushed to the cloud)
```
roscore
rosrun data service service.py
rqt -p barc/perspectives/experiment.perspective
```
When the odroid connects to the Internet through a wifi network, all data collected
will be automatically pushed to the cloud, where the data will be publicly accessible. To view the data, go to [this site](http://dator.forge9.com/) and search for your user name.

## Useful commands
### linux commands / tools
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

### ROS commands
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

## Useful resources

Recommended reading and resources
+ [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/), by Jason M. O'Kane, and this [basic tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) for writing ROS nodes in python
+ [Julia JuMP](https://jump.readthedocs.io/en/latest/), Mathematical Programming using the Julia programming language
+ [Git - the simple guide](http://rogerdudler.github.io/git-guide/), or for a more in-depth reading, consider [this](https://www.atlassian.com/git/tutorials/ ) tutorial by Atlassian
+ [Electronic Speed Control (ESC) manual](http://propeleris.lt/failai/wp-s10c-rtr_manual.pdf), which details how to calibrate and program the ESC
+ [Mass moment of inertia calculation](http://www.mathworks.com/company/newsletters/articles/improving-mass-moment-of-inertia-measurements.html), a MATLAB newsletter explaining how to estimate the mass moment of inertia


## Student projects
+ [Lane Keeping and Obstacle Avoidance](https://github.com/ych09041/me131lane), with [ demo](https://www.youtube.com/watch?v=5HKu7AaSsoM), by Tony Abdo, Hohyun Song, Cheng Hao Yuan, UC Berkeley ME 131, Spring 2016

## Potential issues
### Bad IMU magnetometer readings
The magnetic field around car, perhaps from the motor or aluminum deck, may interfere with the magnetometer readings, meaning the roll, pitch, yaw measurements may be off

### Wrong internal clock time / date
The time and date settings on the odroid may be incorrect. This may be because the [Real Time Clock (RTC) battery](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G137508214939) is not connected or has a loose connection. To update the date / time settings, ensure the RTC battery is firmly connected, and ensure you are connected to the Internet via wifi or ethernet. Next, open a terminal and run the following network time protocol (ntp) commands
```
sudo service ntp stop
sudo ntpdate -s time.nist.gov
sudo service ntp start
```
If you have a "no server suitable for synchronization found", your hosting provider may be blocking ntp packets. Refer to [this](http://askubuntu.com/questions/429306/ntpdate-no-server-suitable-for-synchronization-found) community question for more information
