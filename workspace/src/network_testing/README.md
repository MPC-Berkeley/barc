How to prepare BARC over a network
==================================

The setup will involve one master car, on which roscore will be run on. The remaining cars will be "slave cars" and connect to the master car's roscore.
Note that this is in more of an unstable phase, and that these instructions should be followed step-by-step.

## Preliminary steps

This assumes that you have at least one monitor that is connected to one of the BARC cars (e.g. direct HDMI connection, NoMachine, etc.).

1. Make sure all cars have unique hostnames.
2. Connect all cars on the same network. Ideally, the cars will do this automatically, but sometimes that is not the case. A quick and easy check to see if they are on the same network is to ping the other cars. If they do not respond, that means that they are not connected.
3. Once all the cars have been confirmed to be on the same network, connect a monitor to the master car. Setting up is designed to be performed from the master car's monitor.

## Master Car

#### Setup
1. Open a terminal and run roscore on it. Note: this step may be skipped, but it is useful to have one terminal dedicated to monitoring the ros master.
2. Open a new terminal and change the director to ../network_testing. This can be accomplished with "roscd network_testing".
3. In the terminal, run the command "source ./rosbarc_network -m". This will set the proper environment variables and begin the serial node in the proper namespace.

#### Explanation/Potential bugs
-   Since the scripts will change the environment variables, it is necessary to source the script ([explanation](//askubuntu.com/questions/53177/bash-script-to-set-environment-variables-not-working)). I have not yet figured out how to, if it's possible at all, to use source with the rosrun command. Therefore, instead of running the command "rosrun network_testing rosbarc_network -m", we have to run the command more directly from within the terminal.
-   The directory containing the scripts is not added to the system path, making it necessary to run the script as "./rosbarc_network" instead of "rosbarc_network". This could be updated quite simply if necessary.
- If "rosbarc_network -m" is ran before roscore, it will automatically start the ROS master in the background, and it is possible to continue working as it is. However, since it is in the background, there will be no other way to stop the ROS master but by closing the terminal. If the master is in its own terminal, it can be stopped with "Ctrl-C".

## Slave Car

#### Setup
1. Open a new terminal. Change the directory to network_testing and run the command "./sshcar [SLAVECAR]", where [SLAVECAR] is replaced by the hostname of the car. This script will ssh into the desired car and set up the necessary environment variables. Note: since we don't need to use source for this script, you can also run the script with "rosrun network_testing sshcar [SLAVECAR]".
2. If a prompt for the password is requested, type in the password for the slave car's odroid (generally just "odroid").
3. Change the directory to network_testing and run the command "./rosbarc_network -s". This will begin the serial node in the proper namespace. Note: we also do not require source for this script, so you can also run it with "rosrun network_testing rosbarc_network -s".
4. You can always exit the ssh by typing "exit".

#### Explanation/Potential bugs
-   Review the "Master Car" section's "Explanation/Potential Bugs".
-   The ssh terminal is running in a child process of the terminal, and not the current one. Therefore, source is not necessary.
-   There are many options for sshcar (check the different commands with "sshcar -h"). However, since the script is still unstable, it is highly recommended that you only use the syntax "sshcar [SLAVECAR]".
-   If sshcar could not resolve the hostname, the slave car is either not connected to the network, or the hostname is incorrect.


## Contact

If you find a bug, or have any questions, please contact Kegan Kawamura (kgnkwmr@berkeley.edu).

#### Useful links
-   [Background of using ROS over multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
-   [Using Avahi to resolve hostnames on a network](//wiki.archlinux.org/index.php/avahi#Hostname_resolution)
-   Important environment variables: [ROS_MASTER_URI](//wiki.ros.org/ROS/EnvironmentVariables#ROS_MASTER_URI), [ROS_IP/ROS_HOSTNAME](//wiki.ros.org/ROS/EnvironementVariables#ROS_IP.2BAC8-ROS_HOSTNAME)
-   [Running analogous nodes on different machines at the same time](//wiki.ros.org/Nodes#Remapping_Arguments.A.22Pushing_Down.22)
