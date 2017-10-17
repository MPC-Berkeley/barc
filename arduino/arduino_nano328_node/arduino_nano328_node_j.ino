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
    uint8_t getRCThrottle();
    uint8_t getRCSteering();
    int getEncoderFL();
    int getEncoderFR();
    int getEncoderBL();
    int getEncoderBR();
    //Interrupt service routines
    void incFR();
    void incFL();
    void incBR();
    void incBL();
    void calcThrottle();
    void calcSteering();
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
    // TODO are these the real limits?
    const int MOTOR_MAX = 165;
    const int MOTOR_MIN = 40;
    const int MOTOR_NEUTRAL = 90;
    const int SERVO_OFFSET = 0;

    const int MOTOR_MAX_MS = 1965;
    const int MOTOR_MIN_MS = 996;
    const int MOTOR_NEUTRAL_MS = 1482;
    const int MOTOR_DIFF_MS = (MOTOR_MAX_MS - MOTOR_MIN_MS);

    const int SERVO_MAX_MS = 1963;
    const int SERVO_MIN_MS = 980;
    const int SERVO_NEUTRAL_MS = 1500;
    const int SERVO_DIFF_MS = (SERVO_MAX_MS - SERVO_MIN_MS);

    // Steering limits
    // TODO seems to me that the permissible steering range is really about [58,
    // 120] judging from the sound of the servo pushing beyond a mechanical limit
    // outside that range. The offset may be 2 or 3 deg and the d_theta_max is then
    // ~31.
    const int THETA_CENTER = 90;
    const int THETA_MAX = 150;
    const int THETA_MIN = 33;

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

    // Utility functions
    uint8_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

//Boolean keeping track of whether the Arduino has received a signal from the ECU recently

int received_ecu_signal = 0;

// Initialize an instance of the Car class as car
Car car;

// Callback Functions
// These are really sad solutions to the fact that using class member functions
// as callbacks is complicated in C++ and I haven't figured it out. If you can
// figure it out, please atone for my sins.
void ecuCallback(const barc::ECU& ecu) {
  car.writeToActuators(ecu);
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
barc::Encoder servo;

ros::NodeHandle nh;

// ros::Publisher pub_encoder("encoder", &encoder);
// ros::Publisher pub_servo("servo", &servo);
// ros::Publisher pub_rc_inputs("rc_inputs", &rc_inputs);
ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);


/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // Set up encoders, rc input, and actuators
  // car.initEncoders();
  // car.initRCInput();
  car.initActuators();

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics
  // nh.advertise(pub_encoder);
  // nh.advertise(pub_servo);
  // nh.advertise(pub_rc_inputs);
  nh.subscribe(sub_ecu);

  // Arming ESC, 1 sec /delay for arming and ROS
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

  // Kill the motor if there is non ECU signal within the last 1s.
  if ((millis() - ecu_t0) >= 1000) {
      if (!received_ecu_signal) {
          car.killMotor();
      } else {
          received_ecu_signal = 0;
      }
      ecu_t0 = millis();
  }

  if (dt > 50) {
    // car.readAndCopyInputs();

    // TODO make encoder and rc_inputs private properties on car? Then
    // car.publishEncoder(); and car.publishRCInputs();
    // encoder.FL = car.getEncoderFL();
    // encoder.FR = car.getEncoderFR();
    // encoder.BL = car.getEncoderBL();
    // encoder.BR = car.getEncoderBR();
    // pub_encoder.publish(&encoder);

    // rc_inputs.motor = car.getRCThrottle();
    // rc_inputs.servo = car.getRCSteering();
    // pub_rc_inputs.publish(&rc_inputs);

    t0 = millis();
  }

  // servo.FL = analogRead(A0);
  // pub_servo.publish(&servo);

  nh.spinOnce();
  delay(1);
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
  motor.write(MOTOR_NEUTRAL);
  steering.write(THETA_CENTER + SERVO_OFFSET);
  delay(1000);
}

void Car::writeToActuators(const barc::ECU& ecu) {
  float motorCMD = saturateMotor(ecu.motor);
  float servoCMD = saturateServo(ecu.servo + SERVO_OFFSET);
  motor.writeMicroseconds( (uint16_t) (MOTOR_NEUTRAL_MS + (motorCMD-90.0)*(MOTOR_DIFF_MS)/180.0) );
  steering.write(servoCMD);
  delay(15);
  //steering.writeMicroseconds( (uint16_t) (SERVO_NEUTRAL_MS + (servoCMD-90.0)*(SERVO_DIFF_MS)/180.0) );
}

uint8_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  uint16_t pwm = (microseconds - MOTOR_MIN_MS)/MOTOR_DIFF_MS * 180;
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

void Car::killMotor() {
    motor.writeMicroseconds(MOTOR_NEUTRAL_MS);
    steering.write(90 + SERVO_OFFSET);
}



void Car::incFL() {
  FL_count_shared++;
  updateFlagsShared |= FL_FLAG;
}

void Car::incFR() {
  FR_count_shared++;
  updateFlagsShared |= FR_FLAG;
}

void Car::incBL() {
  BL_count_shared++;
  updateFlagsShared |= BL_FLAG;
}

void Car::incBR() {
  BR_count_shared++;
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
    }
    if(updateFlags & FR_FLAG) {
      FR_count = FR_count_shared;
    }
    if(updateFlags & BL_FLAG) {
      BL_count = BL_count_shared;
    }
    if(updateFlags & BR_FLAG) {
      BR_count = BR_count_shared;
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
