# Connecting to the odroid

You can connect to the odroid one of two ways.

### Direction connection

As you did in the section on flashing, you can directly connect the odroid to a monitor using an HDMI cable. 
Plug in additional peripherals, such as a keyboard and mouse, to program the odroid. 

### Wireless connection

To connect wirelessly to the odroid, the router needs to be connected (via ethernet) to the odroid. 
Make sure you use the **LAN** port from the router.
We recommend connecting using either a secure shell (ssh) protocol or an NX protocol.
The NX protocol by NoMachine provides a GUI interface, essentially allowing your laptop to act as a monitor.
With the ssh protocol, you interact with the remote device (i.e. odroid) only through a terminal.
Beginners should use NoMachine (free download [here](https://www.nomachine.com/download) ). 
For those who prefer the terminal, use the Putty utility (Windows, download [here](http://www.putty.org/ )) directly use the `ssh` command (Mac, Linux) 

After downloading the software, turn on the odroid and identify the router's network from your laptop. 
In general, the bottom face of the router has labels for the default wifi SSID and PWD.
Connect to the network and supply the correct credentials. 

To connect using NoMachine, launch the application, click on the identified linux host. 
Write `odroid` for name and `192.168.100.100` for host, then connect to the host. 
Afterward, you should see the a familiar Ubuntu desktop.

To connect via an ssh protocol, open a terminal and run the following, or use Putty
`ssh odroid@192.168.100.100`

Use an SCP client as needed to transfer files between the local and remote computer.
You will need to learn some unix commands such as cd, pwd, ls, to navigate through the filesystem. 
