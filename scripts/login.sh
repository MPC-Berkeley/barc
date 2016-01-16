#!/bin/bash 
# local start up script when opening bash session

source /usr/local/bin/virtualenvwrapper.sh
export WORKON_HOME=$HOME/.virtualenvs


cd /home/odroid/barc/Dator
workon barc
supervisord -c /home/odroid/barc/Dator/supervisor.conf
cd
deactivate
