# install command-line arduino tool
if ! hash ano 2>/dev/null; then
    # download tool
    echo "installing arduino command line tool"
    cd; git clone https://github.com/scottdarch/Arturo
    cd Arturo

    # install tool
    sudo make install; cd; sudo rm -rf Arturo

    # create arduino upload-folder
    cd ~/barc/arduino/
    mkdir .arduino_nano328_node
    cd .arduino_nano328_node

    # initialize space
    ano init -t blink; rm src/sketch.ino
    cp ~/barc/arduino/arduino_nano328_node/arduino_nano328_node.ino src/

    # link to arduino libraries
    rm -r lib
    ln -s ~/sketchbook/libraries lib

    # set avrdude configuration file
    sudo mkdir /etc/avrdude
    sudo cp /etc/avrdude.conf /etc/avrdude/
    cd;
fi
