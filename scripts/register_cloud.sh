#!/bin/bash

export DATOR_SERVER='http://dator.forge9.com';
sudo service cron stop;
rm -f $HOME/cloud.cfg;
source $HOME/barc/scripts/team_name.sh;
python $HOME/barc/workspace/src/data_service/scripts/upload.py ;
export DATOR_SERVER='http://localhost:8000';
sudo service cron start;
