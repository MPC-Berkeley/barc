# 1. Install wifi utilities

# install create_ap
if ! hash create_ap 2>/dev/null; then
    git clone https://github.com/oblique/create_ap.git
    cd create_ap
    sudo make install
    cd ..
    rm -rf create_ap
else
    echo "create_ap already installed"
fi 

# install all necessary utilities
sudo apt-get install libnl-dev
sudo apt-get install iptables

# install modified binaries for hostapd and hostapd_cli
wget w1.fi/releases/hostapd-2.6.tar.gz                                  # get hostapd
git clone https://github.com/pritambaral/hostapd-rtl871xdrv.git         # get hostapd patch
tar xzvf hostapd-2.6.tar.gz                                             # unzip
cd hostapd-2.6/hostapd                                                  # move into build directory
cp defconfig .config                                                    # copy configuration file
make                                                                    # build
cd ..                                                                   # go to previous directory
patch -Np1 -i ../hostapd-rtl871xdrv/rtlxdrv.patch                       # apply patch
echo CONFIG_DRIVER_RTW=y >> hostapd/.config                 # add configuration command
cd hostapd
make

# install binaries on system
if ! hash hostapd_orig 2>/dev/null; then
    sudo mv /usr/sbin/hostapd /usr/sbin/hostapd_orig
    sudo cp hostapd /usr/sbin/hostapd
else
    echo "modified hostapd binary already installed"
fi

if ! hash hostapd_cli_orig 2>/dev/null; then
    sudo mv /usr/sbin/hostapd_cli /usr/sbin/hostapd_cli_orig
    sudo cp hostapd_cli /usr/sbin/hostapd_cli
else
    echo "modified hostapd_cli binary already installed"
fi

# clean up 
cd ../..
rm -rf hostapd-2.6 hostapd-2.6.tar.gz hostapd-rtl871xdrv

# go to barc folder
roscd barc

# remove current network manager
#sudo apt-get install wicd
#sudo apt-get --purge autoremove network-manager

# install boot time configuration file
#sudo cp accesspoint.conf /etc/init/accesspoint.conf

# remove udev rules (later, will assign wifi dongles to wlan0 and wlan1)
#sudo rm /etc/udev/rules.d/70-persistent-net.rules

# 1. reboot
# 2. plug in wifi-dongles
# 3. edit /etc/udev/rules.d/70-persistent-net.rules file and assign the hardware devices to values wlan0 and wlan1
# 4. check wicd preferences and set eth0 and wlan0
# 5. connect wlan0 to a known network
