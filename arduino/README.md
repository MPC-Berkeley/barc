Arduino connections

<img src="https://www.arduino.cc/en/uploads/Main/ArduinoNanoFront_3_lg.jpg" alt="Arduino Nano" width="400" height="250">
<img src="http://blog.huntgang.com/wp-content/uploads/2015/01/Arduino-Nano-V3.jpg" alt="Arduino Nano" width="400" height="250">



Image from 
<a href="https://www.arduino.cc/en/Main/ArduinoBoardNano">arduino.cc</a>
and 
<a href="http://blog.huntgang.com/2015/01/20/arduino-esp8266-tutorial-web-server-monitor-example/">blog.huntgang</a>

<table style="width:100%">
  <tr>
    <th>Component</th>
    <th>Pin</th> 
  </tr>
  <tr>
    <td>Front Left Encoder</td>
    <td>D2</td> 
  </tr>
    <tr>
    <td>Front Right Encoder</td>
    <td>D3</td> 
  </tr>
  <tr>
    <td>Back Left Encoder</td>
    <td>D5</td> 
  </tr>
    <tr>
    <td>Back Right Encoder</td>
    <td>D6</td> 
  </tr>
  <tr>
  <td>Throttle (from RC receiver channel 2) </td>
    <td>D7</td> 
  </tr>
    <tr>
    <td>Steering (from RC receiver channel 1)</td>
    <td>D8</td> 
  </tr>
  <tr>
    <td>Servo</td>
    <td>D9</td> 
  </tr>
  <tr>
    <td>Electronic Speed Control (for 3-phase Motor)</td>
    <td>D10</td> 
  </tr>
  <tr>
    <td>Front Ultrasound</td>
    <td>D14 = A0</td> 
  </tr>
    <tr>
    <td>Back Ultrasound</td>
    <td>D15 = A1</td> 
  </tr>
  <tr>
    <td>Right Ultrasound</td>
    <td>D16 = A2</td> 
  </tr>
    <tr>
    <td>Left Ultrasound</td>
    <td>D17 = A3</td> 
  </tr>
</table>
___
## ROS communication test
To ensure that ROS master can communicate with the arduino node, run the following commands in separate terminal windows
```
roscore
rosrun rosserial_python serial_node.py
```
Your terminal should output connection information similar to the following. If you see any error messages, then something is wrong with the connection, try reflashing the arduino

(barc)odroid@odroid:~$ rosrun rosserial_python serial_node.py

[INFO]  [WallTime: 1469676336.827668] ROS Serial Python Node

[INFO]  [WallTime: 1469676336.843514] Connecting to /dev/ttyUSB0 at 57600 baud

[INFO]  [WallTime: 1469676339.335939] Note: publish buffer size is 280 bytes

[INFO]  [WallTime: 1469676339.338132] Setup publisher on encoder [barc/Encoder]

[INFO]  [WallTime: 1469676339.346496] Setup publisher on rc_inputs [barc/ECU]

[INFO]  [WallTime: 1469676339.358420] Setup publisher on ultrasound [barc/Ultrasound]

[INFO]  [WallTime: 1469676339.374436] Note: subscribe buffer size is 280 bytes

[INFO]  [WallTime: 1469676339.376341] Setup subscriber on ecu_pwm [barc/ECU]



___
## Notes
+ The ultrasound sensor code is commented out. Uncomment it when the sensors have been connected, otherwise serial communication may not work properly
+ Encoders now use software interrupts (formerly hardware interrupts. Install the necessary library by running `source ~/barc/scripts/install_enable_interrupt.sh`
+ The arduino sends commands to the servo and motor using pulse width modulation. These commands are sent using the `write(pwm_angle)` and `writemicroSeconds(us)` member functions of a `Servo` object. Refer to [this article](http://makezine.com/2014/04/23/arduinos-servo-library-angles-microseconds-and-optional-command-parameters/) for more clarification on these functions 

