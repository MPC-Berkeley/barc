if [ ! -d ~/barc/arduino/.arduino_nano328_node ]; then
    mkdir -p ~/barc/arduino/.arduino_nano328_node/src
fi

if [ ! -L ~/barc/arduino/.arduino_nano328_node/lib ]; then
    ln -s ~/sketchbook/libraries ~/barc/arduino/.arduino_nano328_node/lib
fi

cd ~/barc/arduino/.arduino_nano328_node 
cp ../arduino_nano328_node/arduino_nano328_node.ino src/; 
ano clean; ano build -m nano328; 
ano upload -m nano328 -p /dev/ttyUSB0; 
cd -
