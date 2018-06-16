#!/bin/bash

sudo apt-get install software-properties-common
sudo add-apt-repository ppa:x2go/stable
sudo apt-get update
sudo apt-get install x2goserver x2goserver-xsession

sudo service lightdm restart
#sudo reboot now
