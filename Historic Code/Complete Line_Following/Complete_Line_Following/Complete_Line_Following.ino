#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

int blue_led_pin = 6;
int green_led_pin;

int left_motor_pin = 2;
int right_motor_pin = 3;

bool isMoving = false;

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup has begun");
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  pinMode(extreme_right_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(extreme_left_pin, INPUT);
  pinMode(blue_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
}

void loop() {
  Serial.println("Loop has begun");
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
    if (isMoving) {
    digitalWrite(blue_led_pin, HIGH);
    delay(50);
    digitalWrite(blue_led_pin, LOW);
  }
  }
  
}

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