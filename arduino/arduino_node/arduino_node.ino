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
# Kiet Lam (kiet.lam@berkeley.edu)   
# --------------------------------------------------------------------------- */
 
// include libraries
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <Servo.h>
#include "Maxbotix.h"

// Number of encoder counts on tires
// count tick on {FL, FR, BL, BR}
// F = front, B = back, L = left, R = right
volatile int FL_count = 0; 
volatile int FR_count = 0;

//encoder pins: pins 2,3 are hardware interrupts
const int encPinA = 2; 
const int encPinB = 3;

// Actuator pins: 5,6
// <Servo> data type performs PWM
// Declare variables to hold actuator commands
Servo motor;
Servo steering;
const int motorPin = 10;
const int servoPin = 11;
int motorCMD;
int servoCMD;
const int noAction = 0;

// Actuator constraints (servo)
// Not sure if constraints should be active on motor as well
int d_theta_max = 50; 
int theta_center = 90;
int motor_neutral = 90;
int theta_max = theta_center + d_theta_max;
int theta_min = theta_center - d_theta_max;
int motor_max = 120;
int motor_min = 40;

// variable for time
volatile unsigned long dt;
volatile unsigned long t0;

// pin to poll rear wheel encoder 
int pin_BR = 5;   // back right
boolean val, new_val;
int B_count = 0;


ros::NodeHandle nh;

// define global message variables
// Encoder, Electronic Control Unit, Ultrasound
geometry_msgs::Vector3 enc_msg;
geometry_msgs::Vector3 esc_cmd_msg;
std_msgs::Float32MultiArray ultrasound;

ros::Publisher pub_encoder("encoder", &enc_msg);
ros::Publisher pub_ultrasound("ultrasound", &ultrasound);


/**************************************************************************
ESC COMMAND {MOTOR, SERVO} CALLBACK
**************************************************************************/
void messageCb(const geometry_msgs::Vector3& esc_cmd_msg){
  // deconstruct esc message
  motorCMD = saturateMotor( int(esc_cmd_msg.x) );
  servoCMD = saturateServo( int(esc_cmd_msg.y) );
  
  // apply commands to motor and servo
  motor.write( motorCMD );   
  steering.write(  servoCMD );
}
// ECU := Engine Control Unit
ros::Subscriber<geometry_msgs::Vector3> s("ecu", messageCb);

// Set up ultrasound sensors
Maxbotix us_fr(14, Maxbotix::PW, Maxbotix::LV); // front
Maxbotix us_bk(15, Maxbotix::PW, Maxbotix::LV); // back
Maxbotix us_lt(16, Maxbotix::PW, Maxbotix::LV); // left
Maxbotix us_rt(17, Maxbotix::PW, Maxbotix::LV); // right

/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{ 
  // Set up encoder sensors
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);
  attachInterrupt(0, FL_inc, CHANGE); // args = (digitalPintoInterrupt, ISR, mode), mode set = {LOW, CHANGE, RISING, FALLING}, pin 0 = INT0, which is pin D2
  attachInterrupt(1, FR_inc, CHANGE); //pin 1 = INT1, which is pin D3
 
  // Set up actuators
  motor.attach(motorPin);
  steering.attach(servoPin);
  
  // Arming ESC, 1 sec delay for arming
  motor.write(theta_center);
  steering.write(theta_center);
  delay(1000);
  t0 = millis();
  
  // Start ROS node
  nh.initNode();
  nh.subscribe(s);
  
  // initialize ultrasound sensor datatype
  ultrasound.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  ultrasound.layout.dim[0].label = "cm";
  ultrasound.layout.dim[0].size = 4;
  ultrasound.layout.dim[0].stride = 4;
  ultrasound.layout.data_offset = 0;
  ultrasound.data = (float *) malloc(sizeof(float)*4);
  ultrasound.data_length = 4;
  
  // Public data to topics
  nh.advertise(pub_ultrasound);
  nh.advertise(pub_encoder);
}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop()
{  
    // compute time elapsed (in ms)
  dt = millis() - t0;
  
  // publish measurements
  if (dt > 50) {
    // publish encodeer measurement
    enc_msg.x = FL_count;
    enc_msg.y = FR_count;
    enc_msg.z = 0;
    pub_encoder.publish(&enc_msg);
    
    // publish ultra-sound measurement
    ultrasound.data[0] = us_fr.getRange();
    ultrasound.data[1] = us_bk.getRange();
    ultrasound.data[2] = us_lt.getRange();
    ultrasound.data[3] = us_rt.getRange();
    pub_ultrasound.publish(&ultrasound);
    t0 = millis();
  }
  
  new_val = digitalRead(pin_BR);   // read the input pin
  if (val != new_val){
    B_count++;
    val = new_val;
  }
  
  nh.spinOnce();
}

/**************************************************************************
ENCODER COUNTERS
**************************************************************************/
// increment the counters
void FL_inc() { FL_count++; }
void FR_inc() { FR_count++; }

/**************************************************************************
SATURATE MOTOR AND SERVO COMMANDS
**************************************************************************/
int saturateMotor(int x) 
{
  if (x  == noAction ){
    return motor_neutral;
  }
  
  if (x  >  motor_max) {
    x = motor_max; 
  } 
  else if (x < motor_min) {
    x = motor_min;
  }
  return x;
}

int saturateServo(int x) 
{
  
  if (x  == noAction ){
    return theta_center;
  }
  
  if (x  >  theta_max) {
    x = theta_max; 
  } 
  else if (x < theta_min) {
    x = theta_min;
  }
  return x;
}
