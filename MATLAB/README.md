## Description
This folder contains MATLAB code to process ROS [bag](http://wiki.ros.org/Bags) files, the official message storage format for ROS.

## Requirements
In order to use the software, you need to have the [Robotics System Toolbox](http://www.mathworks.com/hardware-support/robot-operating-system.html?requestedDomain=www.mathworks.com) installed. You can check installed toolboxes by running `ver` at the command line

You also need the [ROS Custom Messages toolbox](http://www.mathworks.com/matlabcentral/fileexchange/49810-robotics-system-toolbox-interface-for-ros-custom-messages) in order for MATLAB to recognize custom ROS messages

## Generating custom messages
Before running, ensure that you have the folder named **packages** in the same directory as the `readBag.m` file. The `rosgenmsg` command generates MATLAB interfaces to custom ROS messages/services based on the message / service definitions inside **packages**.

Before running `readBag.m`, uncomment the `rosgenmsg` and run just this command (Highlight the entire command and press F9). Follow the instructions printed to the MATLAB console. You may need to create the javaclasspath.txt file if no such file exists. This command generates new folders and files with the **packages** folder, in `packages\matlab_gen\msggen`

To save this messages types for future MATLAB sessions, uncomment the `addpath` / `savepath` command, replace *path_to_msggen* with `current_directory_path\packages\matlab_gen\msggen`, where *current_directory_path* is the path to the `readBag.m` file on your machine.  

At this point, you can comment out the `rosgenmsg` and `addpath`/`savepath` commands, and run the entire file.

### Notes:
* If you add, delete, or modify any of the message types (defined inside `packages/barc/msg/`), delete the matlabgen folder and rerun the `rosgenmsg` command.
