/* 
ARDUINO NODE 
 */
 
 // include libraries
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <Servo.h>

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
int theta_max = theta_center + d_theta_max;
int theta_min = theta_center - d_theta_max;
int motor_max = 120;
int motor_min = 40;

// variable for time
volatile unsigned long dt;
volatile unsigned long t0;


ros::NodeHandle nh;

// define global message variables (ESC command, encoder reading)
geometry_msgs::Vector3 enc_msg;
geometry_msgs::Vector3 esc_cmd_msg;
ros::Publisher p("enc_data", &enc_msg);


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
ros::Subscriber<geometry_msgs::Vector3> s("esc_cmd", messageCb);


/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{ 
    // Set up interrupt pins
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);
  
  // Set up communication between Arduino and {Motor and Servo}
  motor.attach(motorPin);
  steering.attach(servoPin);
  
  // Interrupt set up, args = (digitalPintoInterrupt, ISR, mode)
  // mode set = {LOW, CHANGE, RISING, FALLING}
  attachInterrupt(0, FL_inc, CHANGE); //pin 0 = INT0, which is pin D2
  attachInterrupt(1, FR_inc, CHANGE); //pin 1 = INT1, which is pin D3
  
  // Start ROS node
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  
  // Arming ESC, 1 sec delay for arming
  motor.write(theta_center);
  steering.write(theta_center);
  delay(1000);
  t0 = millis();
}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop()
{  
    // compute time elapsed (in ms)
  dt = millis() - t0;
  
  // publish velocity estimate approximately every 20 ms  ( 50 Hz )
  if (dt > 20) {
    enc_msg.x = FL_count;
    enc_msg.y = FR_count;
    
    p.publish(&enc_msg);
    t0 = millis();
  }
  
  nh.spinOnce();
}

/**************************************************************************
ENCODER COUNTERS
**************************************************************************/
// increment the counters
void FL_inc() {
  FL_count++;
}

void FR_inc() {
  FR_count++;
}

/**************************************************************************
SATURATE MOTOR AND SERVO COMMANDS
**************************************************************************/
int saturateMotor(int x) 
{
  
  if (x  == noAction ){
    return x;
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
    return x;
  }
  
  if (x  >  theta_max) {
    x = theta_max; 
  } 
  else if (x < theta_min) {
    x = theta_min;
  }
  return x;
}
