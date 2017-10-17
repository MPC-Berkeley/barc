/* ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was
# based on an open source project by Bruce Wootton, with contributions from
# Kiet Lam (kiet.lam@berkeley.edu). The RC Input code was based on sample code
# from http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
# --------------------------------------------------------------------------- */


// include libraries
#include <ros.h>
#include <barc/ECU.h>
#include <Servo.h>


const int SERVO_NEUTRAL = 90;

const int MOTOR_PIN = 10;
const int SERVO_PIN = 11;


Servo motor;
Servo steering;

void ecuCallback(const barc::ECU& ecu) {
  motor.writeMicroseconds(ecu.motor);
  steering.write(SERVO_NEUTRAL);
}

// Global message variables

barc::ECU ecu;
ros::NodeHandle nh;
ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);



/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Initialize actuators
  motor.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.subscribe(sub_ecu);

}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop() {

  nh.spinOnce();
}


