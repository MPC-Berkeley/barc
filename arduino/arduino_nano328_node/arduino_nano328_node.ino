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


/* ---------------------------------------------------------------------------
WARNING:
* Be sure to have all ultrasound sensors plugged in, otherwise the pins may get stuck in
  some float voltage
# --------------------------------------------------------------------------- */

// include libraries
#include <ros.h>
#include <barc/Ultrasound.h>
#include <barc/Encoder.h>
#include <barc/ECU.h>
#include <barc/Z_KinBkMdl.h>
#include <Servo.h>
#include "Maxbotix.h"
#include <EnableInterrupt.h>

// Pin assignments
const int ENC_FL_PIN = 2;
const int ENC_BR_PIN = 3;
const int ENC_BL_PIN = 5;
const int THROTTLE_PIN = 7;
const int STEERING_PIN = 8;
const int motorPin = 10;
const int servoPin = 11;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
// TODO migrate the wheel encoder interrupts to this same framework, possibly
// connect all four encoders
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership
// of the shared ones. To access these in loop we first turn interrupts off
// with noInterrupts we take a copy to use in loop and the turn interrupts back
// on as quickly as possible, this ensures that we are always able to receive
// new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

// Number of encoder counts on tires
// count tick on {FL, FR, BL, BR}
// F = front, B = back, L = left, R = right
volatile int FL_count = 0;
volatile int FR_count = 0;
volatile int BL_count = 0;
volatile int BR_count = 0;


// <Servo> data type performs PWM
// Declare variables to hold actuator commands
Servo motor;
Servo steering;
// we may be leaving performance on the table by using ints here. RC commands
// are sent as 1000-2000 microsend pwms. The increased coarseness of 0-180 pwm
// may be costing us a bit, especially at low speeds?
int motorCMD;
int servoCMD;
const int noAction = 0;

// Global variables for ESC mode management
const float REVERSE_ESC_THRESHOLD = 0.1;
bool forward_mode = true;
float v_est = 0.0;

// Actuator constraints (servo)
// Not sure if constraints should be active on motor as well
// TODO seems to me that the permissible steering range is really about [58,
// 120] judging from the sound of the servo pushing beyond a mechanical limit
// outside that range. The offset may be 2 or 3 deg and the d_theta_max is then
// ~31.
int d_theta_max = 30;
int theta_center = 90;
int motor_neutral = 90;
int theta_max = theta_center + d_theta_max;
int theta_min = theta_center - d_theta_max;
/* int motor_max = 120; */
/* int motor_min = 40; */
// Enforcing smaller values for testing safety
int motor_max = 101;
int motor_min = 75;

// variable for time
volatile unsigned long dt;
volatile unsigned long t0;

ros::NodeHandle nh;

// define global message variables
// Encoder, Electronic Control Unit, Ultrasound
barc::ECU ecu;
barc::ECU rc_inputs;
barc::Encoder encoder;
barc::Ultrasound ultrasound;

ros::Publisher pub_encoder("encoder", &encoder);
ros::Publisher pub_rc_inputs("rc_inputs", &rc_inputs);
ros::Publisher pub_ultrasound("ultrasound", &ultrasound);

uint8_t microseconds2PWM(uint16_t microseconds)
{
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  uint16_t pwm = (microseconds - 1000.0)/1000.0*180;
  return static_cast<uint8_t>(pwm);
}

/**************************************************************************
ESC COMMAND {MOTOR, SERVO} CALLBACK
**************************************************************************/
void messageCb(const barc::ECU& ecu){
  // deconstruct esc message
  motorCMD = saturateMotor( int(ecu.motor_pwm) );
  servoCMD = saturateServo( int(ecu.servo_pwm) );

  // apply commands to motor and servo
  mapAndWriteMotor(motorCMD);
  steering.write( servoCMD );
}
// ECU := Engine Control Unit
ros::Subscriber<barc::ECU> s("ecu", messageCb);

/**************************************************************************
STATE ESTIMATE {x, y, psi, v} CALLBACK
**************************************************************************/
void stateEstimateCallback(const barc::Z_KinBkMdl& state_est){
  // TODO WARNING This will only work with KinBkMdl state estimate, should revert
  // to master arduino code if you are using DynBkMdl
  v_est = state_est.v;
}
ros::Subscriber<barc::Z_KinBkMdl> state_est_sub("state_estimate", stateEstimateCallback);

// Set up ultrasound sensors
/*
Maxbotix us_fr(14, Maxbotix::PW, Maxbotix::LV); // front
Maxbotix us_bk(15, Maxbotix::PW, Maxbotix::LV); // back
Maxbotix us_rt(16, Maxbotix::PW, Maxbotix::LV); // right
Maxbotix us_lt(17, Maxbotix::PW, Maxbotix::LV); // left
*/

/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoder sensors
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  enableInterrupt(ENC_FL_PIN, FL_inc, CHANGE);
  enableInterrupt(ENC_BR_PIN, BR_inc, CHANGE);
  enableInterrupt(ENC_BL_PIN, BL_inc, CHANGE);

  // Set up software interrupts for RC inputs
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  enableInterrupt(THROTTLE_PIN, calcThrottle, CHANGE);
  enableInterrupt(STEERING_PIN, calcSteering, CHANGE);

  // Set up actuators
  // these can be set up as motor.attach(pin, lower microsecond limit, upper
  // microsecond) maybe this affects how arduino translates 0-180 to
  // microseconds though. Would improve safety if we could do it though
  motor.attach(motorPin);
  steering.attach(servoPin);

    // Start ROS node
  nh.initNode();

  // Publish / Subscribe to topics
  nh.advertise(pub_encoder);
  nh.advertise(pub_rc_inputs);
  nh.advertise(pub_ultrasound);
  nh.subscribe(s);
  nh.subscribe(state_est_sub);

  // Arming ESC, 1 sec delay for arming and ROS
  motor.write(motor_neutral);
  steering.write(theta_center);
  delay(1000);
  t0 = millis();

}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn = 1500;
  static uint16_t unSteeringIn = 1500;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    noInterrupts();

    // make local copies
    bUpdateFlags = bUpdateFlagsShared;
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }

    // clear shared update flags and turn interrupts back on
    bUpdateFlagsShared = 0;
    interrupts();
  }

  // compute time elapsed (in ms)
  dt = millis() - t0;

  // publish measurements
  if (dt > 50) {
    // publish encoder measurement

    // TODO may want to wrap this in noInterrupts(); code; interrupts();
    encoder.FL = FL_count;
    encoder.FR = FR_count;
    encoder.BL = BL_count;
    encoder.BR = BR_count;
    pub_encoder.publish(&encoder);

    rc_inputs.motor_pwm = microseconds2PWM(unThrottleIn);
    rc_inputs.servo_pwm = microseconds2PWM(unSteeringIn);
    pub_rc_inputs.publish(&rc_inputs);

    // publish ultra-sound measurement
    /*
    ultrasound.front = us_fr.getRange();
    ultrasound.back = us_bk.getRange();
    ultrasound.right = us_rt.getRange();
    ultrasound.left = us_lt.getRange();
    */
    pub_ultrasound.publish(&ultrasound);
    t0 = millis();
  }

  nh.spinOnce();
}

/**************************************************************************
INTERRUPT SERVICE ROUTINES
**************************************************************************/
void FL_inc() { FL_count++; }
void BR_inc() { BR_count++; }
void BL_inc() { BL_count++; }

void calcThrottle()
{
  if(digitalRead(THROTTLE_PIN) == HIGH)
  {
    // rising edge of the signal pulse, start timing
    ulThrottleStart = micros();
  }
  else
  {
    // falling edge, calculate duration of throttle pulse
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // set the throttle flag to indicate that a new signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

/**************************************************************************
SATURATE MOTOR AND SERVO COMMANDS
**************************************************************************/
int saturateMotor(int x)
{
  if (x  == noAction ){ return motor_neutral; }

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

/**************************************************************************
ADJUST MOTOR COMMANDS ACCORDING TO BRAKE/REVERSE INTENT
**************************************************************************/
void mapAndWriteMotor(int cmd) {
  // Adjust for the dead spot in the ECU
  // Turning this off temporarily for pass thru controller
  // Clearly need some compilation flags or way to easily compile and flash
  // arduino with different versions
  /* cmd = motorMap(cmd); */

  // The car's ECU acts as a four-state state machine for easy drivability with
  // the remote control. Applying the throttle (motorCMD > 90) passes the given
  // pwm to the motor. Applying reverse throttle (motorCMD < 90) only stops the
  // wheels unless the following sequence is performed: send motorCMD < 67
  // (applying full brakes), send motorCMD in [88, 94] while motor is stopped
  // (return to the neutral region), then send motorCMD < 88 to reverse. The
  // forward_mode boolean tracks a simplified version of the state and applies
  // the necessary 66, 90 sequence when needed to switch from foward to reverse
  // mode.
  if ((cmd > 90) && !forward_mode) {
    forward_mode = true;
  } else if ((cmd < 90) && forward_mode && (v_est < REVERSE_ESC_THRESHOLD)) {
    forward_mode = false;
    motor.write(66);
    // 125ms seems to be the minimal delay to allow the ESC to register these
    // signal changes and enter reverse mode
    // Consider the 0.25s cost of switching between forward and reverse when
    // designing controllers
    delay(125);
    motor.write(90);
    delay(125);
  }

  motor.write(cmd);
}

// Bypass [88, 94] dead region in ECU and leave only 90 as neutral
int motorMap(int cmd) {
  if((cmd > 90) && (cmd < 95)) {
    return 95;
  } else if ((cmd > 87) && (cmd < 90)) {
    return 87;
  } else {
    return cmd;
  }
}
