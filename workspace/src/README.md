# Packages
Software in ROS is organized in packages. This ROS workspace contains the following packages
*   barc
    * barc is the main package for autonomous driving with the RC vehicle. This packages includes example controllers, model-based observers, and other utility modules (e.g. filtering, imu driver, etc.) to control the vehicle. 
*   barc_gui
    * barc_gui is a package for a cloud service pluggin using the ROS rqt gui. During experiments, the user can select which actuator/sensor messages (i.e. motor, encoder, imu, ultrasound, etc.) to record to bagfiles. After the experiment finishes, the data inside the bagfile is parsed, formatted, and pushed to the cloud. To view the data, go to [this site](http://dator.forge9.com/) 
*   data_service
    * data_service is a package for the cloud service functionality. It handles the communication details (authentication, POST/GET requests, etc.) between the client (odroid) and the server.
*   rqt_common_plugins
    *  rqt_common_plugins is a sub-package for the ROS rqt package, which allows users to drag and drop userful pluggins into a QT-based graphical user interface. This package is forked from the main rqt_common_plugins package, ROS package wiki [here](http://wiki.ros.org/rqt_common_plugins). This forked version fixes some bugs with the main package 

#### View installed packages
To check which packages are already install on your machine, do

`rospack list-names`

#### Installing new packages
To add a package, use one of the following to methods

1. Use `apt-get` utility
    * `sudo apt-get install ros-indigo-<package_name>`
    * Downloads and installs the package on your system, usually inside `/opt/ros/indigo`. You should not edit these files  
2. Clone package from github repository. From this directory, execute the following command
    * `git clone <git_ROS_package_URL>.git`
    * Downloads and saves a copy of the package to the folder where the command was called. This is safe to edit

For example, to install the high performance AHRS(Attitude Heading Reference System) imu drivers from Hardkernel, do one of the following
* `sudo apt-get install ros-indigo-myahrs-driver`, or 
* `git clone https://github.com/robotpilot/myahrs_driver.git`

Afterward, go back to the `~/barc/workspace` folder and run `catkin_make` to rebuild the workspace packages 