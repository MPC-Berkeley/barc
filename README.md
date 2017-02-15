# Berkeley Autonomous Race Car (barc) Repo

The Berkeley Autonomous Race Car is a development platform for autonomous driving to achieve complex maneuvers such as drifting, lane changes, and obstacle avoidance. A 1/10 scale RC car and an embedded Linux computer make up the hardware platform of the project. This project aims to be fully open-source. The data collection process is cloud-based and brings new dimensions to the Vehicle Dynamics and Control Theory teaching and research world.

This site is home to the repository. The main site for the project is [here](http://www.barc-project.com/).

## Organization
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
1. Get the parts, [list](https://github.com/BARCproject/barc/blob/master/docs/BillofMaterials.md)
2. Flash the odroid, [instructions](https://github.com/BARCproject/barc/blob/master/docs/FlashingEMMC.md) 
3. Assemble the car, [instructions](https://docs.google.com/document/d/1T8O4JhUlw09ALUGPSX7DlSO7Hc7vcKl_ahBeHncMguE/edit?usp=sharing)
4. Charge the battery, [instructions](https://github.com/BARCproject/barc/blob/master/docs/ChargingBattery.md)
5. Connect to the RC remotely, [instructions](https://github.com/BARCproject/barc/blob/master/docs/ConnectingToOdroid.md)

You can also scroll through the material under our docs section to find other useful information

## Useful resources

Recommended reading and resources
+ [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/), by Jason M. O'Kane, and this [basic tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) for writing ROS nodes in python
+ [Julia JuMP](https://jump.readthedocs.io/en/latest/), Mathematical Programming using the Julia programming language
+ [Git - the simple guide](http://rogerdudler.github.io/git-guide/), or for a more in-depth reading, consider [this](https://www.atlassian.com/git/tutorials/ ) tutorial by Atlassian
+ [Electronic Speed Control (ESC) manual](http://propeleris.lt/failai/wp-s10c-rtr_manual.pdf), which details how to calibrate and program the ESC
+ [Open source ESC](http://vedder.se/2015/01/vesc-open-source-esc/), for building your own ESC
+ [Pulse Wide Modulation (PWM)](http://www.egr.msu.edu/classes/ece480/capstone/fall14/group07/PDFs/PWM_Application_Note.pdf), notes from Michigan State ECE480 class, explaining the concept of PWM and its usage in an arduino.
+ [Arduino Servo Library](http://makezine.com/2014/04/23/arduinos-servo-library-angles-microseconds-and-optional-command-parameters/), article by Make magazine, clarifying the usage of some functions within the library
+ [Mass moment of inertia calculation](http://www.mathworks.com/company/newsletters/articles/improving-mass-moment-of-inertia-measurements.html), a MATLAB newsletter explaining how to estimate the mass moment of inertia
+ [Computer Vision lecture series](https://www.youtube.com/watch?v=N_a6IP6KUSE), from Professor Peter Corke aat Queensland University of Technology

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

### Corrupted file system
If for some reason the Odroid's file system becomes corrupt, the Odroid may not be able to boot properly. To fix this, it is necessary to repair the eMMC. Since the Odroid cannot boot, it is necessary to repair the chip externally.

Remove the eMMC from the Odroid, and attach it to the [Transcend USB](dn.odroid.com/homebackup/201407071823125107.jpg). Insert the USB into a computer with a Linux environment. Open the terminal, and make sure that the USB is properly recognized. A useful command for this is `lsblk`. `lsblk` will display the different disks and partitions on the computer.

Once you've confirmed that the USB is recognized and which name it has (it will likely be /dev/sdb), we need to check which partition contains the Linux OS, which is where the corrupted filesystem likely is. To do this, run the command `sudo fdisk -l` in the terminal. This should display information similiar to `lsblk`. Under the eMMC memory, there should be a partition with "Linux" as its System type (likely to be /dev/sdb2). Now run `fsck [PARTITION]` (e.g. `fsck /dev/sdb2`). This should give you several prompts, and should automatically fix the file system. If it tells you that you do not have the permission to do this, try `sudo fsck [PARTITION]`.

Once the repair is over, remove the USB and insert the eMMC back into the Odroid. Try booting the Odroid to confirm that the repair was successful.


