How to prepare BARC over a network
==================================

The setup will involve one master car, on which roscore will be run on. The remaining cars will be "slave cars" and connect to the master car's roscore.
Note that this is in more of an unstable phase, and that these instructions should be followed step-by-step.

## Preliminary steps

This assumes that you have at least one monitor that is connected to one of the BARC cars (e.g. direct HDMI connection, NoMachine, etc.).

1. Make sure all cars have unique hostnames. To check and change a Linux computer's hostname, refer to the section "Hostname" below.
2. Connect all cars on the same network. Ideally, the cars will do this automatically, but sometimes that is not the case. A quick and easy check to see if they are on the same network is to ping the other cars. If they do not respond, that means that they are not connected. If you do not know how to ping another car, check out the section "Networking Background" below.
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

### Hostname
To check a computer's hostname, simply run 'hostname' in the bash terminal.

If your hostname is not unique, or you just want to change it so that it is easier to identify, perform the following steps (taken from [here](http://www.cyberciti.biz/faq/ubuntu-change-hostname-command/)):

1. Edit the file /etc/hostname. You can do this by running 'sudo vim /etc/hostname'
2. Delete the old hostname and replace with the new one. Save and close the file.
3. Edit the file /etc/hosts. You can do this by running 'sudo vim /etc/hosts'
4. Replace all occurences of the old hostname with the new one. Save and close the file.
5. Reboot the system so changes take effect. You can do this by running 'sudo reboot'
6. Once the system reboots, confirm that the hostname has changed by running 'hostname'

### Networking Background
To establish communication between the BARC units, we must first make sure that they are on the same Wifi network. Currently, the Wifi network is being emitted by a router attached to the master car, but the origin of the network does not matter as long as all the cars are connected to the same one.

Currently, there is no hardware (e.g. an LED that flashes) on the BARC that indicates that the car is connected to the appropriate network. The most direct way to check whether the BARC unit is on the correct network is to connect it to a monitor and visually check which network it is on, if any. However, this may be time consuming and inconvenient, especially with multiple units. To alleviate this, we can set the network configurations of each unit to automatically connect to the appropriate Wifi network when it detects a signal.

However, even then, this does not always work. To confirm that each unit is on the same network, we can ping each unit from one, central unit.

1. Attach a monitor to one unit of your choosing (master car recommended).
2. Confirm that the unit is on the correct network. If not, manually connect it to the correct one.
3. Open a terminal.
4. To check if a certain unit (which is not the current unit) is connected to the correct network, you can run 'ping [HOSTNAME of foreign unit].local'
5. If 'ping' does not exit and is able to send and receive packets from the foreign unit, the foreign unit is on the correct network. If 'ping' fails, try turning off and on the Wifi antenna on the foreign unit, and ping again after a few seconds. By rebooting the antenna, it may connect to the correct network. As a last resort, you may need to connect a monitor to the foreign unit to manually connect to the appropriate network.
6. Repeat steps 4 and 5 on each unit that you want to use.

######Clarification on usage of Ping
Normally, you would need to type the IP address of the foreign machine in order to ping it.

'ping [IP ADDRESS]'

However, we want to keep the process as general as possible. IP addresses may change when cars reconnect to a network, and we do not want to have to keep checking (or remembering!) the IP addresses of each unit being used for the BARC experiment. In order to alleviate this, Avahi has made it possible to substitute a machine's hostname for the IP address (there is a link that explains this below). Essentially, we can use "[HOSTNAME].local" as a substitute for IP addresses in most commands. This has been utilized throughout the development of this software, including ssh.


## Contact

If you find a bug, or have any questions, please contact Kegan Kawamura (kgnkwmr@berkeley.edu).

#### Useful links
-   [Background of using ROS over multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
-   [Using Avahi to resolve hostnames on a network](//wiki.archlinux.org/index.php/avahi#Hostname_resolution)
-   Important environment variables: [ROS_MASTER_URI](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_MASTER_URI), [ROS_IP/ROS_HOSTNAME](http://wiki.ros.org/ROS/EnvironementVariables#ROS_IP.2BAC8-ROS_HOSTNAME)
-   [Running analogous nodes on different machines at the same time](http://wiki.ros.org/Nodes#Remapping_Arguments.A.22Pushing_Down.22)
