#!/bin/bash
# local start up script when opening bash session

# set environment variable for python 
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export VIRTUALENVWRAPPER_VIRTUALENV_ARGS='--no-site-packages'
source /usr/local/bin/virtualenvwrapper.sh

# set environment variables for ROS
export ROS_HOME=$HOME/.ros
source $HOME/barc/workspace/devel/setup.bash

# set path to julia
export PATH=$PATH:/home/odroid/julia/bin

# set environment variables for Amazon Web Server
export DATOR_SERVER='http://localhost:8000';
source $HOME/barc/scripts/team_name.sh

# define commands
#   * nanorst           - resets the arduino nano from the command line (assuming the device is connected and on port /dev/ttyUSB0
#   * rebuild_system    - rebuild all the ROS packages 
alias flash_nano='source ~/barc/scripts/flash_nano.sh'
alias rebuild_system='source ~/barc/scripts/rebuild_system.sh'
alias tmux='tmux -2'
alias reset_wifi_rules='sudo rm /etc/udev/rules.d/70-persistent-net.rules'
alias reset_database='source ~/barc/scripts/reset_database.sh'
alias set_init_ap='sudo cp $HOME/barc/scripts/config/accesspoint.conf /etc/init/accesspoint.conf'
alias set_rc_local='sudo cp $HOME/barc/scripts/config/rc.local /etc/rc.local'
alias register_to_cloud='source ~/barc/scripts/register_cloud.sh'
alias upload_to_cloud='sudo chown -R odroid:odroid $HOME/data_rep; python ~/barc/workspace/src/data_service/scripts/upload.py'
alias rebuild_arduino_lib='cd $HOME/sketchbook/libraries; rm -rf ros_lib; rosrun rosserial_arduino make_libraries.py .; cd -'
alias register_user_locally='reset_database; source $HOME/barc/scripts/team_name.sh; roslaunch data_service service.launch'

# set configuration script for vim text editor
cp ~/barc/scripts/vimrc ~/.vimrc


# added git branch listing (michael)
parse_git_branch() {
     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}
export PS1="\u@\h \[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ "

# work on barc environment
workon barc
