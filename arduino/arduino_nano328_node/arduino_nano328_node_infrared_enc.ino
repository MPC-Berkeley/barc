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


/*
THINGS TO CHECK BEFORE RUNNING THE SCRIPT:
- Check if all Encoders are used in velocity estimation
- Check how many tiks each tire has per revolution (default = 4)

*/

/*
- Room for improvement: Determine velocity over a wider range of encoder tiks (not just one)

*/

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
    void initActuators();
    void armActuators();
    void writeToActuators(const barc::ECU& ecu);
    void reloadCapacitor();
    void determineDischargeTime();
    void evaluateDischargeTime();
    
    // Interrupt service routines
    void incFR();
    void incFL();
    void incBR();
    void incBL();
    float getVelocityEstimate();
    float vel_FL = 0;
    float vel_FR = 0;
    float vel_BL = 0;
    float vel_BR = 0;

  private:
    // Pin assignments
    const int ENC_FL_PIN = 2;
    const int ENC_FR_PIN = 3;
    const int ENC_BL_PIN = 5;
    const int ENC_BR_PIN = 4;
    const int MOTOR_PIN = 10;
    const int SERVO_PIN= 11;

    // Car properties
    // unclear what this is for
    const float noAction = 0.0;

    // Motor limits
    const float MOTOR_MAX = 120.0;
    const float MOTOR_MIN = 40.0;
    const float MOTOR_NEUTRAL = 90.0;

    // Steering limits
    // TODO seems to me that the permissible steering range is really about [58,
    // 120] judging from the sound of the servo pushing beyond a mechanical limit
    // outside that range. The offset may be 2 or 3 deg and the d_theta_max is then
    // ~31.
    const float D_THETA_MAX = 30.0;
    const float THETA_CENTER = 90.0;
    const float THETA_MAX = THETA_CENTER + D_THETA_MAX;
    const float THETA_MIN = THETA_CENTER - D_THETA_MAX;

    // Interfaces to motor and steering actuators
    Servo motor;
    Servo steering;

    // Number of encoder counts on tires
    // count tick on {FL, FR, BL, BR}
    // F = front, B = back, L = left, R = right
    int FL_count = 0;
    int FR_count = 0;
    int BL_count = 0;
    int BR_count = 0;

    // Delta time withing two magnets                                  			    //(ADDED BY TOMMI 7JULY2016)
    // F = front, B = back, L = left, R = right                         			//(ADDED BY TOMMI 7JULY2016)
    unsigned long FL_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long FR_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long BL_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long BR_new_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)

    unsigned long FL_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long FR_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long BL_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)
    unsigned long BR_old_time = 0;                                         //(ADDED BY TOMMI 7JULY2016)

    unsigned long FL_DeltaTime = 0;                                         			        //(ADDED BY TOMMI 7JULY2016)
    unsigned long FR_DeltaTime = 0;                                          			        //(ADDED BY TOMMI 7JULY2016)
    unsigned long BL_DeltaTime = 0;                                           			    //(ADDED BY TOMMI 7JULY2016)
    unsigned long BR_DeltaTime = 0;                                            			    //(ADDED BY TOMMI 7JULY2016)
                     
    // Time it takes for the capacitor of encoder to discharge
    unsigned long FL_DCTime = 0;
    unsigned long FR_DCTime = 0;
    unsigned long BL_DCTime = 0;
    unsigned long BR_DCTime = 0;
    
    // Binary variable to represent last seen surface of encoder
    int FL_state = 1; //1: White surface, 0: black surface
    int FR_state = 1; //1: White surface, 0: black surface
    int BL_state = 1; //1: White surface, 0: black surface
    int BR_state = 1; //1: White surface, 0: black surface

    // Utility functions
    uint8_t microseconds2PWM(uint16_t microseconds);
    float saturateMotor(float x);
    float saturateServo(float x);
};

// Initialize an instance of the Car class as car
Car car;

// Callback Functions
// These are really sad solutions to the fact that using class member functions
// as callbacks is complicated in C++ and I haven't figured it out. If you can
// figure it out, please atone for my sins.
void ecuCallback(const barc::ECU& ecu) {
  car.writeToActuators(ecu);
}


// Variables for time step
unsigned long dt;
unsigned long t0;

// Global message variables
// Encoder, RC Inputs, Electronic Control Unit, Ultrasound
barc::ECU ecu;
barc::Vel_est vel_est;
std_msgs::Float32 debug_val;

ros::NodeHandle nh;


// Define publishers
ros::Subscriber<barc::ECU> sub_ecu("ecu_pwm", ecuCallback);
ros::Publisher pub_vel_est("vel_est", &vel_est);          // vel est publisher
ros::Publisher pub_debug("debug", &debug_val);          // vel est publisher

/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
  // What is with Serial.begin(9600); ?
  
  // Set up encoders, rc input, and actuators
  //car.initEncoders();
  //car.initRCInput();
  car.initActuators();

  // Start ROS node
  nh.initNode();

  // Publish and subscribe to topics  OLD METHOD
//  nh.advertise(pub_encoder);
  //nh.advertise(pub_encoder_dt_FL);                                                    //(ADDED BY TOMMI 7JULY2016)
  //nh.advertise(pub_encoder_dt_FR);                                                    //(ADDED BY TOMMI 7JULY2016)
  //nh.advertise(pub_encoder_dt_BL);                                                    //(ADDED BY TOMMI 7JULY2016)
  //nh.advertise(pub_encoder_dt_BR);                                                    //(ADDED BY TOMMI 7JULY2016)
  //nh.advertise(pub_rc_inputs);
  //nh.advertise(pub_ultrasound);
  
  // NEW ENCODER METHOD
  nh.advertise(pub_vel_est);
  nh.advertise(pub_debug);
  nh.subscribe(sub_ecu);

  // Arming ESC, 1 sec delay for arming and ROS
  car.armActuators();
  t0 = millis();

}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop() {
  // compute time elapsed (in ms)
  dt = millis() - t0;


  if (dt > 50) {
    //car.readAndCopyInputs();

    vel_est.vel_est = car.getVelocityEstimate();
    vel_est.vel_fl = car.vel_FL;  // velocity - front left
    vel_est.vel_fr = car.vel_FR; // front right
    vel_est.vel_bl = car.vel_BL;  // back left
    vel_est.vel_br = car.vel_BR;  //back right
    vel_est.header.stamp = nh.now();
    pub_vel_est.publish(&vel_est);               // publish estimated velocity
    ////////////////////////////////////////////////!!!!

    t0 = millis();
  }
  
  
  else{
    // Evaluate information from encoders
    car.reloadCapacitor();
    car.determineDischargeTime();
    car.evaluateDischargeTime();
  }
  
  
  pub_debug.publish(&debug_val);  

  nh.spinOnce();
}

/**************************************************************************
CAR CLASS IMPLEMENTATION
**************************************************************************/
void Car::reloadCapacitor() {
  pinMode(ENC_FL_PIN, OUTPUT);
  pinMode(ENC_FR_PIN, OUTPUT);
  digitalWrite(ENC_FL_PIN, HIGH);
  digitalWrite(ENC_FR_PIN, HIGH);
  
  // Pauses the program for the amount of time (in microseconds) specified as parameter.
  delayMicroseconds(5);
}

void Car::determineDischargeTime() {
  
  // Determine discharge time for encoder at FL
  pinMode(ENC_FL_PIN, INPUT);
  long time = micros();
  
  while (digitalRead(ENC_FL_PIN) == HIGH && micros() - time < 2000);
    FL_DCTime = micros() - time;

  
  // Determine discharge time for encoder at FR
  pinMode(ENC_FR_PIN, INPUT);
  time = micros();
  while (digitalRead(ENC_FR_PIN) == HIGH && micros() - time < 2000);
    FR_DCTime = micros() - time;
 }

void Car::evaluateDischargeTime() {
  debug_val.data = BL_state - 1;
  
  // ---- FL ENCODER
  // If surface is now black and was white before
  if (FL_DCTime > 900 && FL_state == 1){
    incFL();
    FL_state = 0;
  }else if (FL_DCTime < 600 && FL_state == 0){
     FL_state = 1; 
  }
  
    // ---- FR ENCODER
  if (FR_DCTime > 900 && FR_state == 1){
    incFR();
    FR_state = 0;
  }else if (FR_DCTime < 600 && FR_state == 0){
     FR_state = 1; 
  }
  
  // --- BL ENCODER
    if (BL_DCTime > 900 && BL_state == 1){
      incBL();
      BL_state = 0;
    }else if (BL_DCTime < 600 && BL_state == 0){
       BL_state = 1; 
    }
    
   // --- BR ENCODER
    if (BR_DCTime > 900 && BR_state == 1){
      incBR();
      BR_state = 0;
    }else if (BR_DCTime < 600 && BR_state == 0){
       BR_state = 1; 
    }
 }
 
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
  motor.writeMicroseconds( (uint16_t) (1500.0 + (motorCMD-90.0)*1000.0/180.0) );
  steering.write(servoCMD);
}

uint8_t Car::microseconds2PWM(uint16_t microseconds) {
  // Scales RC pulses from 1000 - 2000 microseconds to 0 - 180 PWM angles
  uint16_t pwm = (microseconds - 1000.0)/1000.0*180;
  return static_cast<uint8_t>(pwm);
}


void Car::incFL() {
  FL_count++;
  FL_old_time = FL_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  FL_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  FL_DeltaTime = FL_new_time - FL_old_time;
}

void Car::incFR() {
  FR_count++;
  FR_old_time = FR_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  FR_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  FR_DeltaTime = FR_new_time - FR_old_time;

}

void Car::incBL() {
  BL_count++;
  BL_old_time = BL_new_time;                                                    //(ADDED BY TOMMI 7JULY2016)
  BL_new_time = micros();      // new instant of passing magnet is saved         //(ADDED BY TOMMI 7JULY2016)
  BL_DeltaTime = BL_new_time - BL_old_time;

}

void Car::incBR() {
  BR_count++;
  BR_old_time = BR_new_time;                                                   //(ADDED BY TOMMI 7JULY2016)
  BR_new_time = micros();      // new instant of passing magnet is saved        //(ADDED BY TOMMI 7JULY2016)
  BR_DeltaTime = BR_new_time - BR_old_time;

}

float Car::getVelocityEstimate() {
  long numTiksPerTire = 4.0;
  vel_FL = 0.0;
  vel_FR = 0.0;
  vel_BL = 0.0;
  vel_BR = 0.0;
  if(FL_DeltaTime > 0){
    vel_FL = 2.0*3.141593*0.036/numTiksPerTire*1.0/FL_DeltaTime*1000000.0;
  }
  if(FR_DeltaTime > 0){
    vel_FR = 2.0*3.141593*0.036/numTiksPerTire*1.0/FR_DeltaTime*1000000.0;
  }
  if(BL_DeltaTime > 0){
    vel_BL = 2.0*3.141593*0.036/numTiksPerTire*1.0/BL_DeltaTime*1000000.0;
  }
  if(BR_DeltaTime > 0){
    vel_BR = 2.0*3.141593*0.036/numTiksPerTire*1.0/BR_DeltaTime*1000000.0;
  }
  return ( vel_FL + vel_FR + vel_BL + vel_BR) / 4.0;  
}