#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(2);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  pinMode(2,INPUT);
  pinMode(3, INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);

}

void loop() {
  int extreme_right = digitalRead(2);
  int right = digitalRead(3);
  int left = digitalRead(4);
  int extreme_left = digitalRead(5);
  Serial.print(extreme_right);
  Serial.print(right);
  Serial.print(left);
  Serial.println(extreme_right);
  if(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0 ){
  Motor_Right->run(FORWARD);
  Motor_Left->run(FORWARD);
  }
  
  else if(extreme_right == 0  && right == 1 && left == 1 && extreme_left == 1){
    delay(250);

    Motor_Left->run(RELEASE);

    delay(500);
    
  }

  

}
