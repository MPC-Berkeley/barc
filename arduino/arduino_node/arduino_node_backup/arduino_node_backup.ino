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

int QRE_Value = 0;
int QRE_Value2 = 0;
int currentState = 0;
int currentState2 = 0;

/*
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
*/
// variable for time
volatile unsigned long dt;
volatile unsigned long t0;

/*
// pin to poll rear wheel encoder 
int pin_BR = 5;   // back right
boolean val, new_val;
int B_count = 0;
*/

ros::NodeHandle nh;

// define global message variables (ESC command, encoder reading)
geometry_msgs::Vector3 enc_msg;
//geometry_msgs::Vector3 esc_cmd_msg;
ros::Publisher p("enc_data", &enc_msg);


/**************************************************************************
ESC COMMAND {MOTOR, SERVO} CALLBACK
**************************************************************************/
/*
void messageCb(const geometry_msgs::Vector3& esc_cmd_msg){
  // deconstruct esc message
  motorCMD = saturateMotor( int(esc_cmd_msg.x) );
  servoCMD = saturateServo( int(esc_cmd_msg.y) );
  
  // apply commands to motor and servo
  motor.write( motorCMD );   
  steering.write(  servoCMD );
}
// ECU := Engine Control Unit
ros::Subscriber<geometry_msgs::Vector3> s("ecu_cmd", messageCb);

*/
/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/

void setup()
{ 
  
  // Set up communication between Arduino and {Motor and Servo}
  //motor.attach(motorPin);
  //steering.attach(servoPin);
  
  // Start ROS node
  nh.initNode();
  //nh.subscribe(s);
  nh.advertise(p);
  
  // Arming ESC, 1 sec delay for arming
  //motor.write(theta_center);
  //steering.write(theta_center);
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
  
  readQD(QRE_Value,QRE_Value2);
  
  if (QRE_Value > 900 && currentState == 1)
  {
    FR_inc();
    currentState = 0;
  }
   
   if (QRE_Value < 600 && currentState == 0)
   {
     currentState = 1;
   }
   
     if (QRE_Value2 > 900 && currentState2 == 1)
  {
    FL_inc();
    currentState2 = 0;
  }
   
   if (QRE_Value2 < 600 && currentState2 == 0)
   {
     currentState2 = 1;
   }
   
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
READ FUNCTION
**************************************************************************/

void readQD(int& QRE_Value,int& QRE_Value2) {
   //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  
  // Configures the specified pin to behave either as an input or an output -- pinMode(pin, mode)
  pinMode(encPinA, OUTPUT);
   pinMode(encPinB, OUTPUT);

  // Write a HIGH or a LOW value to a digital pin. -- digitalWrite(pin, value), to emit light
  digitalWrite(encPinA, HIGH);
  digitalWrite(encPinB, HIGH);
  
  // Pauses the program for the amount of time (in microseconds) specified as parameter.
  delayMicroseconds(5);
  // set pin back as input to receive data
  pinMode(encPinA, INPUT);

  // Returns the number of microseconds since the Arduino board began running the current program
  long time = micros();

  // Reads the value from a specified digital pin, either HIGH or LOW. -- digitalRead(pin)
  // measure time for the capacitor to discharge, go to low, the more reflected light(white surface) the faster
  while (digitalRead(encPinA) == HIGH && micros() - time < 2000);
  int diff = micros() - time;


  QRE_Value = diff;
  
  // second encoder
  pinMode(encPinB, INPUT);
  time = micros();
  while (digitalRead(encPinB) == HIGH && micros() - time < 2000);
  diff = micros() - time;
  
  QRE_Value2 = diff;
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
/*
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
*/
