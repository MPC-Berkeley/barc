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
//#include <barc/Ultrasound.h>
#include <barc/Vel_est.h>
#include <barc/ECU.h>
#include <Servo.h>
#include <EnableInterrupt.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

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
    uint8_t getRCThrottle();
    uint8_t getRCSteering();
    int getEncoderFL();
    int getEncoderFR();
    int getEncoderBL();
    int getEncoderBR();
    unsigned long getEncoder_dTime_FL();                                          //(ADDED BY TOMMI 7JULY2016)
    unsigned long getEncoder_dTime_FR();                                          //(ADDED BY TOMMI 7JULY2016)
    unsigned long getEncoder_dTime_BL();                                          //(ADDED BY TOMMI 7JULY2016)
    unsigned long getEncoder_dTime_BR();                                          //(ADDED BY TOMMI 7JULY2016)
    // Interrupt service routines
    void incFR();
    void incFL();
    void incBR();
    void incBL();
    void calcThrottle();
    void calcSteering();
    float getVelocityEstimate();
    void killMotor();
    float vel_FL = 0;
    float vel_FR = 0;
    float vel_BL = 0;
    float vel_BR = 0;
    const int FEEDBACK_PIN = A1;

  private:
    // Pin assignments
    const int ENC_FL_PIN = 2;
    const int ENC_FR_PIN = 3;
    const int ENC_BL_PIN = 4;
    const int ENC_BR_PIN = 5;
    const int THROTTLE_PIN = 7;
    const int STEERING_PIN = 8;
    const int MOTOR_PIN = 10;
    const int SERVO_PIN= 11;

    // Car properties
    // unclear what this is for
    const float noAction = 0.0;

    // Motor limits
    // TODO are these the real limits?
    const float MOTOR_MAX = 150.0;
    const float MOTOR_MIN = 40.0;
    const float MOTOR_NEUTRAL = 89.0;
    // Optional: smaller values for testing safety
    /* const int MOTOR_MAX = 100; */
    /* const int MOTOR_MIN = 75; */

    // Steering limits
    // TODO seems to me that the permissible steering range is really about [58,
    // 120] judging from the sound of the servo pushing beyond a mechanical limit
    // outside that range. The offset may be 2 or 3 deg and the d_theta_max is then
    // ~31.
    const float D_THETA_MAX = 50.0;
    const float THETA_CENTER = 90.0;
    const float THETA_MAX = THETA_CENTER + D_THETA_MAX;
    const float THETA_MIN = THETA_CENTER - D_THETA_MAX;

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

    uint32_t throttleStart;
    uint32_t steeringStart;
    volatile uint16_t throttleInShared;
    volatile uint16_t steeringInShared;
    uint16_t throttleIn = 1500;
    uint16_t steeringIn = 1500;
    
    float throttle_neutral_ms = 1400.0;
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

    // Delta time withing two magnets                                           //(ADDED BY TOMMI 7JULY2016)
    // F = front, B = back, L = left, R = right                               //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long FL_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long FR_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long BL_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long BR_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)

    volatile unsigned long FL_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long FR_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long BL_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    volatile unsigned long BR_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)

    unsigned long FL_DeltaTime = 0;                                                       //(ADDED BY TOMMI 7JULY2016)
    unsigned long FR_DeltaTime = 0;                                                       //(ADDED BY TOMMI 7JULY2016)
    unsigned long BL_DeltaTime = 0;                                                     //(ADDED BY TOMMI 7JULY2016)
    unsigned long BR_DeltaTime = 0;                                                     //(ADDED BY TOMMI 7JULY2016)


    // Utility functions
    uint8_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

// Boolean keeping track of whether the Arduino has received a signal from the ECU recently
int received_ecu_signal = 0;

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
barc::Vel_est vel_est;
// std_msgs::Int32 servo_feedback;

ros::NodeHandle nh;

ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);
// ros::Publisher pub_servo_feedback("servo_feedback", &servo_feedback); 
ros::Publisher pub_vel_est("vel_est", &vel_est);          // vel est publisher

int min_degrees;
int max_degrees;
int min_feedback;
int max_feedback;
int tolerance = 2;


void calibrate(Servo servo, int analog_pin, int min_pos, int max_pos) {
	servo.write(min_pos);
	min_degrees = min_pos;
	delay(2000);
	min_feedback = analogRead(analog_pin);

	servo.write(max_pos);
	max_degrees = max_pos;
	delay(2000);
	max_feedback = analogRead(analog_pin);	
}



/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoders, rc input, and actuators
  car.initEncoders();
  car.initActuators();

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  // nh.advertise(pub_servo_feedback);
  nh.advertise(pub_vel_est);
  nh.subscribe(sub_ecu);

  // calibrate(car.steering, car.FEEDBACK_PIN, 80, 120);
	
  // car.steering.write(90);
  // delay(2000);

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
  if( (millis() - ecu_t0) >= 200){
    if(!received_ecu_signal){
        car.killMotor();
    } else{
        received_ecu_signal = 0;
    }
    ecu_t0 = millis();
  }

  if (dt > 50) {
    car.readAndCopyInputs();

    vel_est.vel_est = car.getVelocityEstimate();
    vel_est.vel_fl = car.vel_FL;
	// vel_est.vel_fl = analogRead(car.FEEDBACK_PIN);  // definitely not the ideal way of doing this!!!!
    vel_est.vel_fr = car.vel_FR;
    vel_est.vel_bl = car.vel_BL;
    vel_est.vel_br = car.vel_BR;
    vel_est.header.stamp = nh.now();
    pub_vel_est.publish(&vel_est);       
	
	// servo_feedback.data = analogRead(car.FEEDBACK_PIN);
    // pub_servo_feedback.publish(&servo_feedback);
  }

  nh.spinOnce();
}

/**************************************************************************
CAR CLASS IMPLEMENTATION
**************************************************************************/
float Car::saturateMotor(float x) {
  if (x  == noAction ){ return MOTOR_NEUTRAL; }

  if (x  >  MOTOR_MAX) {
    x = MOTOR_MAX;
  } else if (x < MOTOR_MIN) {
    x = MOTOR_MIN;
  }
  return x;
}

float Car::saturateServo(float x) {
  if (x  == noAction ) {
    return THETA_CENTER;
  }

  if (x  >  THETA_MAX) {
    x = THETA_MAX;
  }
  else if (x < THETA_MIN) {
    x = THETA_MIN;
  }
  return x;
}

void Car::initEncoders() {
  pinMode(ENC_FR_PIN, INPUT_PULLUP);
  pinMode(ENC_FL_PIN, INPUT_PULLUP);
  pinMode(ENC_BR_PIN, INPUT_PULLUP);
  pinMode(ENC_BL_PIN, INPUT_PULLUP);
  enableInterrupt(ENC_FR_PIN, incFRCallback, FALLING);   //enables interrupts from Pin ENC_FR_PIN, when signal changes (CHANGE). And it call the function 'incFRCallback'
  enableInterrupt(ENC_FL_PIN, incFLCallback, FALLING);
  enableInterrupt(ENC_BR_PIN, incBRCallback, FALLING);
  enableInterrupt(ENC_BL_PIN, incBLCallback, FALLING);
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
  motor.write(MOTOR_NEUTRAL);
  steering.write(THETA_CENTER);
  delay(1000);
}

void Car::writeToActuators(const barc::ECU& ecu) {
  float motorCMD = saturateMotor(ecu.motor);
  float servoCMD = saturateServo(ecu.servo);
  motor.writeMicroseconds( (uint16_t) (1500 + (motorCMD-90.0)*1000.0/180.0));
  steering.write(servoCMD);
}

uint8_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  uint16_t pwm = (microseconds - 1000.0)/1000.0*180;  return static_cast<uint8_t>(pwm);
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

void Car::killMotor() {
    motor.writeMicroseconds( (uint16_t) throttle_neutral_ms );
    steering.writeMicroseconds( (uint16_t) servo_neutral_ms );
}

void Car::incFL() {
  FL_count_shared++;
  FL_old_time = FL_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  FL_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  updateFlagsShared |= FL_FLAG;
}

void Car::incFR() {
  FR_count_shared++;
  FR_old_time = FR_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  FR_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  updateFlagsShared |= FR_FLAG;
}

void Car::incBL() {
  BL_count_shared++;
  BL_old_time = BL_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  BL_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  updateFlagsShared |= BL_FLAG;
}

void Car::incBR() {
  BR_count_shared++;
  BR_old_time = BR_new_time;                                                   //(ADDED BY TOMMI 7JULY2016)
  BR_new_time = micros();      // new instant of passing magnet is saved        //(ADDED BY TOMMI 7JULY2016)
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
      FL_DeltaTime = FL_new_time - FL_old_time;                              //(ADDED BY TOMMI 7JULY2016)
    }
    if(updateFlags & FR_FLAG) {
      FR_count = FR_count_shared;
      FR_DeltaTime = FR_new_time - FR_old_time;                             //(ADDED BY TOMMI 7JULY2016)
    }
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
      BL_DeltaTime = BL_new_time - BL_old_time;                              //(ADDED BY TOMMI 7JULY2016)
    }
    if(updateFlags & BR_FLAG) {
      BR_count = BR_count_shared;
      BR_DeltaTime = BR_new_time - BR_old_time;                              //(ADDED BY TOMMI 7JULY2016)
    }
    // clear shared update flags and turn interrupts back on
    updateFlagsShared = 0;
    interrupts();
  }
}

uint8_t Car::getRCThrottle() {
  return microseconds2PWM(throttleIn);
}
uint8_t Car::getRCSteering() {
  return microseconds2PWM(steeringIn);
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

unsigned long Car::getEncoder_dTime_FL() {                               //(ADDED BY TOMMI 7JULY2016)
  return FL_DeltaTime;                                         //(ADDED BY TOMMI 7JULY2016)
}                                                              //(ADDED BY TOMMI 7JULY2016)
unsigned long Car::getEncoder_dTime_FR() {                               //(ADDED BY TOMMI 7JULY2016)
  return FR_DeltaTime;                                         //(ADDED BY TOMMI 7JULY2016)
}                                                              //(ADDED BY TOMMI 7JULY2016)+
unsigned long Car::getEncoder_dTime_BL() {                               //(ADDED BY TOMMI 7JULY2016)
  return BL_DeltaTime;                                         //(ADDED BY TOMMI 7JULY2016)
}                                                              //(ADDED BY TOMMI 7JULY2016)
unsigned long Car::getEncoder_dTime_BR() {                               //(ADDED BY TOMMI 7JULY2016)
  return BR_DeltaTime;                                         //(ADDED BY TOMMI 7JULY2016)
}                                                              //(ADDED BY TOMMI 7JULY2016)

float Car::getVelocityEstimate() {
  vel_FL = 0.0;
  vel_FR = 0.0;
  vel_BL = 0.0;
  vel_BR = 0.0;
  if(FL_DeltaTime > 0){
    vel_FL = 2.0*3.141593*0.036/2.0*1.0/FL_DeltaTime*1000000.0;
  }
  if(FR_DeltaTime > 0){
    vel_FR = 2.0*3.141593*0.036/2.0*1.0/FR_DeltaTime*1000000.0;
  }
  if(BL_DeltaTime > 0){
    vel_BL = 2.0*3.141593*0.036/2.0*1.0/BL_DeltaTime*1000000.0;
  }
  if(BR_DeltaTime > 0){
    vel_BR = 2.0*3.141593*0.036/2.0*1.0/BR_DeltaTime*1000000.0;
  }
  return ( vel_FL + vel_FR ) / 2.0;
}
