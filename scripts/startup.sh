#!/bin/bash 
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

touch /home/odroid/testestestestest


source /home/odroid/.local/bin/virtualenvwrapper.sh
export WORKON_HOME=$HOME/.virtualenvs


cd /home/odroid/barc/Dator
workon barc
supervisord -c /home/odroid/barc/Dator/supervisor.conf
