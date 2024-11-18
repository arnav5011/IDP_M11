#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

int blue_led_pin = 6;
int green_led_pin = 7;

int left_motor_pin = 2;
int right_motor_pin = 3;

bool isMoving = false;

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
  // put your setup code here, to run once:
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
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);

  // turn on motor
  Motor_Left->run(RELEASE);
  Motor_Right->run(RELEASE);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

}

void loop() {
  Serial.println("Loop has begun");
  int extreme_right_pin = 2;
  int right_pin = 3;
  int left_pin = 4;
  int extreme_left_pin = 5;
  int extreme_right = digitalRead(extreme_right_pin);
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);
  Serial.print(extreme_left);
  Serial.print(left);
  Serial.print(right);
  Serial.println(extreme_right);

  while(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){
    Serial.println("Moving Forward");
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    isMoving = true;

    if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){ //Off coursed to rightside so turn to left
      Motor_Left->setSpeed(200);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){ //Off coursed to leftside so turn to right
      Motor_Right->setSpeed(200);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
      Motor_Right->run(BACKWARD);
      """while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
        """
      delay(250);
      
    }
    else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){ //Turn Left
      Motor_Left->run(BACKWARD);
      """while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }"""
      delay(250);
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==1){
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      isMoving = false;
    }
    else if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      isMoving = false;
    }
    if (isMoving) {
    digitalWrite(blue_led_pin, HIGH);
    delay(50);
    digitalWrite(blue_led_pin, LOW);
  }
  }
  
}
"""
void move() {
  int extreme_right = digitalRead(extreme_right_pin);
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);
  Serial.print(extreme_left);
  Serial.print(left);
  Serial.print(right);
  Serial.print(extreme_right);
  while(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){
    Serial.println("Moving Forward");
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    isMoving = true;

    if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){ //Off coursed to rightside so turn to left
      Motor_Left->setSpeed(200);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){ //Off coursed to leftside so turn to right
      Motor_Right->setSpeed(200);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
      Motor_Right->run(BACKWARD);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){ //Turn Left
      Motor_Left->run(BACKWARD);
      while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==1){
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      isMoving = false;
    }
    else if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      isMoving = false;
    }
  }

}
"""