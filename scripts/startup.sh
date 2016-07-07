#!/bin/bash
# local start up script when opening bash session


export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export VIRTUALENVWRAPPER_VIRTUALENV_ARGS='--no-site-packages'
source /usr/local/bin/virtualenvwrapper.sh
workon barc

source /home/odroid/team_name.sh

alias nanorst='cd ~/barc/arduino/.arduino_nano328_node; cp ../arduino_nano328_node/arduino_nano328_node.ino src/; ano clean; ano build -m nano328; ano upload -m nano328 -p /dev/ttyUSB0; roscd barc'
alias rebuild_system='source ~/barc/scripts/rebuild_system.sh'

export ROS_HOME=$HOME/barc/workspace/src/barc/rosbag/

cp ~/barc/scripts/vimrc ~/.vimrc
