#!/bin/bash

sudo apt-get install vino

dconf write /org/gnome/desktop/remote-access/enabled true
dconf write /org/gnome/desktop/remote-access/prompt-enabled false
dconf write /org/gnome/desktop/remote-access/require-encryption false

echo "[Desktop Entry]
Type=Application
Exec=/usr/lib/vino/vino-server
Hidden=false
X-MATE-Autostart-enabled=true
Name[en]=Vino Server
Name=Vino Server
Comment[en]=VNC Remote Desktop Server
Comment=VNC Remote Desktop Server" > ~/.config/autostart/vino-server.desktop

echo "[SeatDefaults]
autologin-user=odroid
autologin-user-timeout=0" > /etc/lightdm/lightdm.conf

sudo service lightdm restart
#sudo reboot now
