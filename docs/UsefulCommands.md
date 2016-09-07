Below is a list of linux and ROS commands that you may find helpful when working with the barc platform

## linux commands / tools
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

## ROS commands
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