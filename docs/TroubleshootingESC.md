# Troubleshooting the ESC

Thanks to Vince Viola for writing this troubleshooting guide

#### Blinking red LED: ESC calibration issue 
If the ESC blinks red and fails to respond to any command, do the following:
 
1. Check Arduino for burnt out ports
   * Switch ESC and servo ports
     1. If ESC LED stops blinking red, and it can receive motor commands, but not servo commands, then that port on the Arduino is probably damaged. Do not go through with the rest of the troubleshooting.
     2. If ESC LED continues blinking red, calibrate the ESC
2. Calibrate ESC:
   1. Unplug servo and ESC cable from Arduino, and plug them into the 2.4 GHz receiver. Servo connects to channel 1 (CH1) and ESC connects to channel 2 (CH2). Black wire faces toward car center, white faces torward outside.
   2. Turn off ESC
   3. Set remote control to the following settings below (other configurations may work), and then turn it on
      * Set ST (steering)  to NOR (normal). In normal mode, turning the steering input frontward turns the steering wheel to the right
      * Set TH (throttle)  to NOR (normal)
      * Set TH.D/R (sensitivity) set to 10
      * Set ST.D/R (sensitivity) set to 0
      * Set nobs on TH.TRIM and ST.TRIM (offsets) to line up with the white arrow
      * Set top switch on the side set to G
      * Set bottom switch on the side near the trigger to NP
   4. Hold down ESC set button, turn on ESC (while still holding set), and then immediately release the set button when the red LED light begins blinking. LED should continuously beep and blink red
      * If a big red light on the 2.4 GHz transceiver pulsing (not the red LED on the ESC), that means you do not have connection to your remote control.  Turn it off and on a few times or move to an area with less wireless interference.
   5. Perform calibration procecure
      * Set throttle and steering both in neutral position (if not already) and press set button. ESC will beep and green LED will blink once
      * Pull back throttle all the way and press set button again.  ESC will beep and green LED will blink twice
      * Push the throttle all the way forward and press set button again.  ESC will beep and green LED will blink three times
      * Release the throttle.  Green LED on ESC will blink and beep two more times.  
      * After 3 seconds, you should be able to control the car with your remote control.  Your ESC should be successfully calibrated.  

#### Resetting ESC to default settings

1. Perform the steps in "Calibrating ESC"
2. Turn on ESC, and hold down set button until you see both the green LED and the red LED blinking many times.  The red LED should then start to blink again on its own to show that the ESC is not calibrated.
3. Perform the steps in "Calibrating ESC"
4. ESC should now have default settings 

#### Blinking green LED: Low battery issue

If there is a blinking green LED, then most likely the issue is with a low battery.  In addition, alert tones are used to diagnose the problem.  

ALERT TONES:

1. “beep-beep-, beep-beep-, beep-beep-” with a 1s time interval between "beep-beep-" tones. 
   * This is an input voltage abnormal alert tone. This tone will be sounded when the input voltage of the ESC is not in the normal range.

2. "beep-, beep-, beep-" with a 2s time interval between "beep-" tones.
   * This is a throttle signal abnormal alert tone. This tone will be sounded whenever the normal throttle signal cannot be detected by the ESC.  
 
#### Summary
If you can control the car with the remote control, but cannot send commands to the ESC with ROS, then the Arduino may be damaged.  Try reflashing or replacing the Arduino.

If you are unable to control the car with the remote control in any way
  * Ensure the ESC and Servo are connected to the right channels (Servo to CH1, ESC to CH2)
  * Ensure the 2.4 GHz receiver has a connection (e.g. the big square red light is not pulsing on your 2.4 GHz receiver)

If the above methods fail, the ESC may be damaged, consider replacing it.
