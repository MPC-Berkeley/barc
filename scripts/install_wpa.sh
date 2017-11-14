# 1. remove wifi dongles

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
sudo apt-get install wicd
sudo apt-get --purge autoremove network-manager

# install modified binaries for hostapd and hostapd_cli
sudo apt-get install libnl-3-dev
sudo apt-get install libnl-genl-3-dev
cd Downloads
wget w1.fi/releases/hostapd-2.6.tar.gz                     # get hostapd
git clone https://github.com/pritambaral/hostapd-rtl871xdrv # get hostapd patch
tar xzvf hostapd-2.6.tar.gz                                 # unzip
cd hostapd-2.6/hostapd                                      # move into build directory
cp defconfig .config                                        # copy configuration file
patch -Np1 -i ../../hostapd-rtl871xdrv/rtlxdrv.patch           # apply patch
echo CONFIG_DRIVER_RTW=y >> hostapd/.config                 # add configuration command
make

# install binaries on system
if ! hash hostapd 2>/dev/null; then
    sudo cp hostapd /usr/sbin/hostapd
else
	if ! hash hostapd_orig 2>/dev/null; then
		sudo mv /usr/sbin/hostapd /usr/sbin/hostapd_orig
		sudo cp hostapd/hostapd /usr/sbin/hostapd
	else
		echo "modified hostapd binary already installed"
	fi
fi

if ! hash hostapd_cli 2>/dev/null; then
    sudo cp hostapd_cli /usr/sbin/hostapd_cli
else
	if ! hash hostapd_orig 2>/dev/null; then
		sudo mv /usr/sbin/hostapd_cli /usr/sbin/hostapd_cli_orig
		sudo cp hostapd_cli /usr/sbin/hostapd_cli
	else
		echo "modified hostapd binary already installed"
	fi
fi

# clean up 
cd ../..
rm -rf hostapd-2.6 hostapd-2.6.tar.gz hostapd-rtl871xdrv

# install boot time configuration file
#sudo cp accesspoint.conf /etc/init/accesspoint.conf

# remove udev rules (later, will assign wifi dongles to wlan0 and wlan1)
#sudo rm /etc/udev/rules.d/70-persistent-net.rules

# 1. reboot
# 2. plug in wifi-dongles
# 3. edit /etc/udev/rules.d/70-persistent-net.rules file and assign the hardware devices to values wlan0 and wlan1
# 4. check wicd preferences and set eth0 and wlan0
# 5. connect wlan0 to a known network
