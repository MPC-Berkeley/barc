Arduino connections

<img src="https://www.arduino.cc/en/uploads/Main/ArduinoNanoFront_3_lg.jpg" alt="Arduino Nano" width="600" height="450">
Image from 
<a href="https://www.arduino.cc/en/Main/ArduinoBoardNano">arduino.cc</a>

<table style="width:100%">
  <tr>
    <th>Component</th>
    <th>Pin</th> 
  </tr>
  <tr>
    <td>Sero</td>
    <td>D11</td> 
  </tr>
  <tr>
    <td>Motor</td>
    <td>D10</td> 
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

Notes
<ul>
  <li>The arduino nano only uses two of the four encoders because it has only two interrupts</li>
  <li>The ultrasound sensor code is commented out. Uncomment it when the sensors have been connected, otherwise serial communication may not work </li>
  <li> To rebuild arduiono-ROS libraries, open a terminal and run <br> <font face="Digital, Arial, Helvetica, sans-serif">source ~/barc/scripts/rebuild_system.sh</font> 
  <br> and then reflash the arduino</li>
</ul>
