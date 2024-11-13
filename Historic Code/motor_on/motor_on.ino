#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);




void setup() {

  int extreme_left_pin = 2;
  int left_pin = 3;
  int right_pin = 4;
  int extreme_right_pin = 5;
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(255);
  myMotor2->setSpeed(255);

  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);

  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);

}

void loop() {
  int extreme_left = 0;
  int left = digitalRead(3);
  int right = digitalRead(4);
  int extreme_right = digitalRead(5);
  Serial.print(extreme_left);
  Serial.print(left);
  Serial.print(right);
  Serial.println(extreme_right);
  if (true) {

    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
  }
  else {
    
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);

  }

 delay(100);
  
}