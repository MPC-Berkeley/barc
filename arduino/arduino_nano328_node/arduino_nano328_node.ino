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
#include <barc/Encoder.h>
#include <barc/ECU.h>
#include <Servo.h>
#include <EnableInterrupt.h>

/**************************************************************************
CAR CLASS DEFINITION (would like to refactor into car.cpp and car.h but can't figure out arduino build process so far)
**************************************************************************/
class Car {
  public:
    void initEncoders();
    void initRCInput();
    void initActuators();
    void armActuators();
    void writeToActuators(const barc::ECU& ecu);
    // Used for copying variables shared with interrupts to avoid read/write
    // conflicts later
    void readAndCopyInputs();
    // Getters
    uint16_t getRCThrottle();
    uint16_t getRCSteering();
    int getEncoderFL();
    int getEncoderFR();
    int getEncoderBL();
    int getEncoderBR();
    float getVelEstFL();
    float getVelEstFR();
    float getVelEstBL();
    float getVelEstBR();

    // Interrupt service routines
    void incFR();
    void incFL();
    void incBR();
    void incBL();
    void calcThrottle();
    void calcSteering();
    void calcVelocityEstimate();
    void killMotor();
  private:
    // Pin assignments
    const int ENC_FL_PIN = 2;
    const int ENC_FR_PIN = 3;
    const int ENC_BL_PIN = 5;
    const int ENC_BR_PIN = 6;
    const int THROTTLE_PIN = 7;
    const int STEERING_PIN = 8;
    const int MOTOR_PIN = 10;
    const int SERVO_PIN= 9;

    // Car properties
    // unclear what this is for
    const int noAction = 0;

    // Motor limits
    // TODO  fix limits?
    const int MOTOR_MAX = 2000;
    const int MOTOR_MIN = 800;
    const int MOTOR_NEUTRAL = 1500;
    const int THETA_CENTER = 1500;
    const int THETA_MAX = 1900;
    const int THETA_MIN = 1100;

    // Interfaces to motor and steering actuators
    Servo motor;
    Servo steering;

    // Utility variables to handle RC and encoder inputs
    volatile uint8_t updateFlagsShared;
    uint8_t updateFlags;
    const int THROTTLE_FLAG = 1;
    const int STEERING_FLAG = 2;
    const int FL_FLAG = 3;
    const int FR_FLAG = 4;
    const int BL_FLAG = 5;
    const int BR_FLAG = 6;

    // RC joystick control variables
    uint32_t throttleStart;
    uint32_t steeringStart;
    volatile uint16_t throttleInShared;
    volatile uint16_t steeringInShared;
    uint16_t throttleIn = 1500;
    uint16_t steeringIn = 1500;
 
    // motor / servo neutral state (milliseconds)
    float throttle_neutral_ms = 1500.0;
    float servo_neutral_ms = 1500.0;

    // Number of encoder counts on tires
    // count tick on {FL, FR, BL, BR}
    // F = front, B = back, L = left, R = right
    volatile int FL_count_shared = 0;
    volatile int FR_count_shared = 0;
    volatile int BL_count_shared = 0;
    volatile int BR_count_shared = 0;
    int FL_count = 0;
    int FR_count = 0;
    int BL_count = 0;
    int BR_count = 0;
    int FL_count_old = 0;
    int FR_count_old = 0;
    int BL_count_old = 0;
    int BR_count_old = 0;
    float vel_FL = 0;
    float vel_FR = 0;
    float vel_BL = 0;
    float vel_BR = 0;


    // Timing parameters
    // F = front, B = back, L = left, R = right   
    volatile unsigned long FL_new_time = 0;
    volatile unsigned long FR_new_time = 0;
    volatile unsigned long BL_new_time = 0;
    volatile unsigned long BR_new_time = 0;
    volatile unsigned long FL_old_time = 0; 
    volatile unsigned long FR_old_time = 0;
    volatile unsigned long BL_old_time = 0;
    volatile unsigned long BR_old_time = 0;
    unsigned long FL_DeltaTime = 0;
    unsigned long FR_DeltaTime = 0;
    unsigned long BL_DeltaTime = 0;
    unsigned long BR_DeltaTime = 0;

    // Utility functions
    uint16_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

// Boolean keeping track of whether the Arduino has received a signal from the ECU recently
int received_ecu_signal = 0;
float pi                = 3.141593;
float R                 = 0.051;        // radius of the wheel

// Initialize an instance of the Car class as car
Car car;

// Callback Functions
// These are really sad solutions to the fact that using class member functions
// as callbacks is complicated in C++ and I haven't figured it out. If you can
// figure it out, please atone for my sins.
void ecuCallback(const barc::ECU& ecu) {
  car.writeToActuators(ecu);
  received_ecu_signal = 1;
}
void incFLCallback() {
  car.incFL();
}
void incFRCallback() {
  car.incFR();
}
void incBLCallback() {
  car.incBL();
}
void incBRCallback() {
  car.incBR();
}
void calcSteeringCallback() {
  car.calcSteering();
}
void calcThrottleCallback() {
  car.calcThrottle();
}

// Variables for time step
volatile unsigned long dt;
volatile unsigned long t0;
volatile unsigned long ecu_t0;

// Global message variables
// Encoder, RC Inputs, Electronic Control Unit, Ultrasound
barc::ECU ecu;
barc::ECU rc_inputs;
barc::Encoder encoder;
barc::Encoder vel_est;

ros::NodeHandle nh;

ros::Publisher pub_encoder("encoder", &encoder);
ros::Publisher pub_vel_est("vel_est", &vel_est); 
ros::Publisher pub_rc_inputs("rc_inputs", &rc_inputs);
ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);

/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoders, rc input, and actuators
  car.initEncoders();
  car.initRCInput();
  car.initActuators();

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  nh.advertise(pub_encoder);
  nh.advertise(pub_rc_inputs);
  nh.advertise(pub_vel_est);
  nh.subscribe(sub_ecu);

  // Arming ESC, 1 sec delay for arming and ROS
  car.armActuators();
  t0 = millis();
  ecu_t0 = millis();

}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop() {
  // compute time elapsed (in ms)
  dt = millis() - t0;

  // kill the motor if there is no ECU signal within the last 1s
  if( (millis() - ecu_t0) >= 1000){
    if(!received_ecu_signal){
        car.killMotor();
    } else{
        received_ecu_signal = 0;
    }
    ecu_t0 = millis();
  }

  if (dt > 50) {
    car.readAndCopyInputs();

    // publish velocity estimate
    car.calcVelocityEstimate();
    vel_est.FL  = car.getVelEstFL();
    vel_est.FR  = car.getVelEstFR();
    vel_est.BL  = car.getVelEstBL();
    vel_est.BR  = car.getVelEstBR();
    pub_vel_est.publish(&vel_est); 

    // publish encoder ticks
    encoder.FL = car.getEncoderFL();
    encoder.FR = car.getEncoderFR();
    encoder.BL = car.getEncoderBL();
    encoder.BR = car.getEncoderBR();
    pub_encoder.publish(&encoder);

    rc_inputs.motor = car.getRCThrottle();
    rc_inputs.servo = car.getRCSteering();
    pub_rc_inputs.publish(&rc_inputs);

    t0 = millis();
  }

  nh.spinOnce();
}

/**************************************************************************
CAR CLASS IMPLEMENTATION
**************************************************************************/
float Car::saturateMotor(float x) {
  if (x > MOTOR_MAX) { x = MOTOR_MAX; }
  if (x < MOTOR_MIN) { x = MOTOR_MIN; }
  return x;
}

float Car::saturateServo(float x) {
  if (x > THETA_MAX) { x = THETA_MAX; }
  if (x < THETA_MIN) { x = THETA_MIN; }
  return x;
}

void Car::initEncoders() {
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  enableInterrupt(ENC_FR_PIN, incFRCallback, CHANGE);
  enableInterrupt(ENC_FL_PIN, incFLCallback, CHANGE);
  enableInterrupt(ENC_BR_PIN, incBRCallback, CHANGE);
  enableInterrupt(ENC_BL_PIN, incBLCallback, CHANGE);
}

void Car::initRCInput() {
  pinMode(THROTTLE_PIN, INPUT_PULLUP);
  pinMode(STEERING_PIN, INPUT_PULLUP);
  enableInterrupt(THROTTLE_PIN, calcThrottleCallback, CHANGE);
  enableInterrupt(STEERING_PIN, calcSteeringCallback, CHANGE);
}

void Car::initActuators() {
  motor.attach(MOTOR_PIN);
  steering.attach(SERVO_PIN);
}

void Car::armActuators() {
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
  delay(1000);
}

void Car::writeToActuators(const barc::ECU& ecu) {
  motor.writeMicroseconds( (uint16_t) saturateMotor( ecu.motor ) );
  steering.writeMicroseconds( (uint16_t) saturateServo( ecu.servo ) );
}

uint16_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  // Mapping from microseconds to pwm angle
  // 0 deg -> 1000 us , 90 deg -> 1500 us , 180 deg -> 2000 us
  // ref: camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino

  // saturate signal
  if(microseconds > 2000 ){ microseconds = 2000; }
  if(microseconds < 1000 ){ microseconds = 1000; }

  // map signal from microseconds to pwm angle
  uint16_t pwm = (microseconds - 1000.0)/1000.0*180;
  return static_cast<uint8_t>(pwm);
}

void Car::calcThrottle() {
  if(digitalRead(THROTTLE_PIN) == HIGH) {
    // rising edge of the signal pulse, start timing
    throttleStart = micros();
  } else {
    // falling edge, calculate duration of throttle pulse
    throttleInShared = (uint16_t)(micros() - throttleStart);
    // set the throttle flag to indicate that a new signal has been received
    updateFlagsShared |= THROTTLE_FLAG;
  }
}

void Car::calcSteering() {
  if(digitalRead(STEERING_PIN) == HIGH) {
    steeringStart = micros();
  } else {
    steeringInShared = (uint16_t)(micros() - steeringStart);
    updateFlagsShared |= STEERING_FLAG;
  }
}

void Car::killMotor(){
  motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
  steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
}

void Car::incFL() {
  FL_count_shared++;
  FL_old_time = FL_new_time; 
  FL_new_time = micros();
  updateFlagsShared |= FL_FLAG;
}

void Car::incFR() {
  FR_count_shared++;
  FR_old_time = FR_new_time;
  FR_new_time = micros();
  updateFlagsShared |= FR_FLAG;
}

void Car::incBL() {
  BL_count_shared++;
  BL_old_time = BL_new_time;
  BL_new_time = micros();
  updateFlagsShared |= BL_FLAG;
}

void Car::incBR() {
  BR_count_shared++;
  BR_old_time = BR_new_time;
  BR_new_time = micros();
  updateFlagsShared |= BR_FLAG;
}

void Car::readAndCopyInputs() {
  // check shared update flags to see if any channels have a new signal
  if (updateFlagsShared) {
    // Turn off interrupts, make local copies of variables set by interrupts,
    // then turn interrupts back on. Without doing this, an interrupt could
    // update a shared multibyte variable while the loop is in the middle of
    // reading it
    noInterrupts();
    // make local copies
    updateFlags = updateFlagsShared;
    if(updateFlags & THROTTLE_FLAG) {
      throttleIn = throttleInShared;
    }
    if(updateFlags & STEERING_FLAG) {
      steeringIn = steeringInShared;
    }
    if(updateFlags & FL_FLAG) {
      FL_count = FL_count_shared;
      FL_DeltaTime = FL_new_time - FL_old_time;
    }
    if(updateFlags & FR_FLAG) {
      FR_count = FR_count_shared;
      FR_DeltaTime = FR_new_time - FR_old_time;
    }
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
      BL_DeltaTime = BL_new_time - BL_old_time;
    }
    if(updateFlags & BR_FLAG) {
      BR_count = BR_count_shared;
      BR_DeltaTime = BR_new_time - BR_old_time;
    }
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    interrupts();
  }
}

uint16_t Car::getRCThrottle() {
  return throttleIn;
}
uint16_t Car::getRCSteering() {
  return steeringIn;
}

int Car::getEncoderFL() {
  return FL_count;
}
int Car::getEncoderFR() {
  return FR_count;
}
int Car::getEncoderBL() {
  return BL_count;
}
int Car::getEncoderBR() {
  return BR_count;
}
float Car::getVelEstFL() {
  return vel_FL;
}
float Car::getVelEstFR() {
  return vel_FR;
}
float Car::getVelEstBL() {
  return vel_BL;
}
float Car::getVelEstBR() {
  return vel_BR;
}

void Car::calcVelocityEstimate() {

    // vel = distance / time
    // distance = 2*pi*R/8 since there are 8 partitions
    if(FL_count_old != FL_count){
        vel_FL = 0.25*pi*R/(FL_DeltaTime/1000000.0);    }
    else{ vel_FL = 0.0; }

    if(FR_count_old != FR_count){
        vel_FR = 0.25*pi*R/(FR_DeltaTime/1000000.0);    }
    else{ vel_FR = 0.0; }

    if(BL_count_old != BL_count){
        vel_BL = 0.25*pi*R/(BL_DeltaTime/1000000.0);    }
    else{ vel_BL = 0.0; }

    if(BR_count_old != BR_count){
        vel_BR = 0.25*pi*R/(BR_DeltaTime/1000000.0);    }
    else{ vel_BR = 0.0; }

    // update history
    FL_count_old = FL_count;
    FR_count_old = FR_count;
    BL_count_old = BL_count;
    BR_count_old = BR_count;
}
