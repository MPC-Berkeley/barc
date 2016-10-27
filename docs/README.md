# Introduction

The BARC vehicle is based on a 1/10 scale remote control (RC) vehicle. The project can be divided into three sections: mechanical design, electrical design, and software design. The mechanical section focuses on the designs for the chassis deck and sensor mounts. The electrical section focuses on the sensor selection, the wiring, and the power distribution. The software section focuses on the data acquisition from sensors and the control algorithms, all within in Robotic Operating System (ROS) based framework.

## Mechanical design
The main RC mechanical components of the vehicle include the chassis, suspension, transmission system, and wheels. We designed an aluminum deck for the purpose of (a) holding the microcontroller and sensors and (b) protecting the on-board electronics.

## Electronic design
The on-board electronics move the vehicle. The standard electronic components in the RC car are listed in the table below.

| Component | Description |
| ----------- | ----------- |
| Servo | Set the steering angle |
| Electronic Speed Control (ESC) | Set the target rotation speed of the motor |
| Switch | Turn the ESC on / off |
| Brushless Motor | Move the car forward or backward, or brake
| Lithium polymer (LiPo) battery | Supply power to on-board electronics |
| Antenna | Get command from the remote control |
| Receiver | Relay command signal from antenna to servo and ESC |


These on-board electronics are shown in the image below, original image [here](http://www.hobbyking.com/hobbyking/store/__84945__Basher_RZ_4_1_10_Rally_Racer_V2_ARR_.html)

<img src="https://github.com/BARCproject/barc/raw/master/docs/imgs/std_electronic_parts.PNG" alt="Drawing" style="width: 400px;"/>

## Software design
The BARC platform uses ROS, an open-source library and set of tools for programming robots. The architecture abstracts low level details, like timing, communication protocols, concurrency, and other processes by providing a simple programming paradigm with a well-written, easy-to-use set of application programming interfaces (APIs). The ROS infrastructure is build on the concepts of nodes, topics, messages. A short, a node is a computer program (i.e. le) that processes some data. That data is then packaged into a message format, and then published, or broadcast, onto a topic, for other nodes to listen to. In other words, ROS uses a many-to-many communication paradigm to decouple nodes, so that each node (or process) can be written independent of the others. For an in-depth explanation,
read [here](http://www.cse.sc.edu/~jokane/agitr/).


---------------------------
## Control Architecture
The control architecture consists of a controller and state observer. The state observer uses sensor measurements (e.g. from encoders, imu, camera, sonar range finder ) and input commands to estimate the vehicle state (e.g. velocity, position, orientation). A high level controller takes that state estimate and outputs a high level command (e.g. desired steering angle, desired velocity). The low level controller translates these high level commands into low level commands to send to the actuators (e.g. motor, servo)

<img src="https://github.com/BARCproject/barc/raw/master/docs/imgs/control_architecture.PNG" alt="Drawing" style="width: 400px;"/>