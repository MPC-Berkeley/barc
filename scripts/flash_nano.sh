if [ ! -d ~/barc/arduino/.arduino_nano328_node ]; then
    mkdir -p ~/barc/arduino/.arduino_nano328_node/src
fi

cd ~/barc/arduino/.arduino_nano328_node 
ln -s ~/sketchbook/libraries lib
cp ../arduino_nano328_node/arduino_nano328_node.ino src/; 
ano clean; ano build -m nano328; 
ano upload -m nano328 -p /dev/ttyUSB0; 
cd -
