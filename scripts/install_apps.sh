# install command-line arduino tool
if ! hash ano 2>/dev/null; then
    # download tool
    echo "installing arduino command line tool"
    cd; git clone https://github.com/scottdarch/Arturo
    cd Arturo

    # install tool
    sudo make install; cd; sudo rm -rf Arturo

    # link to arduino libraries
    if [ ! -d ~/barc/arduino/.arduino_nano328_node ]; then
        mkdir -p ~/barc/arduino/.arduino_nano328_node/src
    fi

    if [ ! -L ~/barc/arduino/.arduino_nano328_node/lib ]; then
        ln -s ~/sketchbook/libraries ~/barc/arduino/.arduino_nano328_node/lib
    fi

    # set avrdude configuration file
    sudo mkdir /etc/avrdude
    sudo cp /etc/avrdude.conf /etc/avrdude/
    cd;
fi

# install EnableInterrupt library for arduino
if [ ! -d $HOME/sketchbook/libraries/EnableInterrupt ]; then
    mkdir -p $HOME/sketchbook/libraries
    cd $HOME/sketchbook/libraries
    git clone https://github.com/GreyGnome/EnableInterrupt.git
fi

# install monokai colors for vim
cd
rm -rf .vim
mkdir -p .vim
cd .vim
git clone https://github.com/sickill/vim-monokai.git
mv vim-monokai/* .
rm -rf vim-monokai/
cd 

# install apps
# * vim     - text editor
# * chrony  - time synchronization utility
# * tree    - file directory visualization utility
# * htop    - interactive process viewer
sudo apt-get install vim chrony tree htop tmux exfat-fuse exfat-utils nmap
