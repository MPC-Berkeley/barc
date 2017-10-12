#!/bin/bash
# local start up script when opening bash session

# set environment variable for python 
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export VIRTUALENVWRAPPER_VIRTUALENV_ARGS='--no-site-packages'
source /usr/local/bin/virtualenvwrapper.sh
workon barc

# set environment variables for ROS
source $HOME/barc/workspace/devel/setup.bash
export ROS_HOME=$HOME/barc/workspace/src/barc/rosbag/

# set environment variables for Amazon Web Server
source $HOME/team_name.sh

# define commands
#   * nanorst           - resets the arduino nano from the command line (assuming the device is connected and on port /dev/ttyUSB0
#   * rebuild_system    - rebuild all the ROS packages 
alias flash_nano='cd ~/barc/arduino/.arduino_nano328_node; cp ../arduino_nano328_node/arduino_nano328_node.ino src/; ano clean; ano build -m nano328; ano upload -m nano328 -p /dev/ttyUSB0; roscd barc'
alias rebuild_system='source ~/barc/scripts/rebuild_system.sh'
alias tmux='tmux -2'
alias reset_wifi_rules='sudo rm /etc/udev/rules.d/70-persistent-net.rules'
alias reset_database='source ~/barc/scripts/reset_database.sh'
alias set_init_ap='sudo cp $HOME/barc/scripts/accesspoint.conf /etc/init/accesspoint.conf'
alias register_cloud='source ~/barc/scripts/register_cloud.sh'

# set configuration script for vim text editor
cp ~/barc/scripts/vimrc ~/.vimrc


# added git branch listing (michael)
parse_git_branch() {
     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}
export PS1="\u@\h \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ "
