#!/bin/bash

sudo service cron stop; 
rm -f ~/barc/Dator/db.sqlite3; 
rm -f ~/default.cfg ; 
python ~/barc/Dator/manage.py syncdb --noinput; 
sudo service cron start;
