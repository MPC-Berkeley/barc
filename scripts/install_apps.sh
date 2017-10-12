# install command-line arduino tool
if ! hash ano 2>/dev/null; then
    # download tool
    echo "installing arduino command line tool"
    cd; git clone https://github.com/scottdarch/Arturo
    cd Arturo

    # install tool
    sudo make install; cd; sudo rm -rf Arturo

    # link to arduino libraries
    rm -r lib
    ln -s ~/sketchbook/libraries lib

    # set avrdude configuration file
    sudo mkdir /etc/avrdude
    sudo cp /etc/avrdude.conf /etc/avrdude/
    cd;
fi

# install EnableInterrupt library for arduino
if [ ! -d $HOME/sketchbook/libraries/EnableInterrupt ]; then
    mkdir -p $HOME/sketchbook/libraries
    cd $HOME/sketchbook/libraries
    wget -O enableinterrupt.zip "https://bintray.com/greygnome/generic/download_file?file_path=enableinterrupt-0.9.5.zip"
    unzip enableinterrupt.zip
    rm enableinterrupt.zip
fi

# install monokai colors for vim
cd
rm -rf .vim
mkdir -p .vim
cd .vim
git init
git remote add origin https://github.com/sickill/vim-monokai.git
git pull origin master
cd 

# install apps
# * vim     - text editor
# * chrony  - time synchronization utility
# * tree    - file directory visualization utility
# * htop    - interactive process viewer
sudo apt-get install vim chrony tree htop tmux exfat-fuse exfat-utils
