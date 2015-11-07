volatile int count = 0; //number of encoder counts (using a quadrature encoder)

//encoder pins: pins 2,3 are hardware interrupts
const int encoderA = 2; 
const int encoderB = 3;

//actuator pins: pins 3,5,6,9,10,11 are capable of hardware pwm
const int motorPWM = 5;
const int servoPWM = 6;

//end flag and delimiter used to parse incoming serial message of setpoints, etc.
//Your message should be of the form: "XXX,YYY,ZZZ,etc.f". Note that there are no spaces.
char flag = 'f';
char delim[2] = ",";

volatile unsigned long curr_time;
volatile unsigned long temp;
char *parser;
char readIn[16];

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(motorPWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), inc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), inc, CHANGE);
  curr_time = millis();
  Serial.begin(250000);
  Serial.println("Running");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  temp = millis()-curr_time;
  if (temp > 2) {
    Serial.println(count/temp * 1000); //prints velocity out to the serial port
    count = 0;
    curr_time = millis();
  }


  //parses any available incoming data. Use atoi() to convert the characters to bytes (for math manipulation)
  temp = Serial.available();
  if (temp > 0) {
    Serial.readBytesUntil(flag, readIn, 8);
    parser = strtok(readIn, delim);
    while (parser != NULL) {
      //Serial.println(atoi(parser) + 2);
      parser = strtok(NULL, delim);
    }
  }
  
  delay(1);
}

void inc() {
  count++;
}
