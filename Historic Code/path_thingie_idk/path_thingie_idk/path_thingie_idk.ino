#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin;
int right_pin;
int left_pin;
int extreme_left_pin;

int blue_led_pin;
int green_led_pin;

int left_motor_pin;
int right_motor_pin;

int count_split = 0; //Checks Split Count
int count_right = 0; //Checks number of juncitons where right turn could be taken
int count_left = 0; //checks number of junctions where left turn could be taken 

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  pinMode(extreme_right_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(extreme_left_pin, INPUT);
  pinMode(blue_led_pin, INPUT);
  pinMode(green_led_pin, INPUT);
}

void loop() {
  
  
  mov_forward_until_split_or_turn();

}

void mov_forward_until_split_or_turn(){
  int extreme_right = digitalRead(extreme_right_pin);
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);

  while(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);

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
    else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){ //Turn Left
      left_count = left_count + 1;
      if (left_count == 1){
        turn_left();
      }
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
      right_count = right_count + 1;
      if (right_count == 2 || right_count == 3){
        turn_right();
      }
    }
    else if(extreme_right==1 && right == 1 && left == 1 && extreme_left == 1) {
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      count_split = count_split + 1;
      if(count_split == 1){
        turn_left();
      }
      else if(count_split == 2){
        turn_right();
      }
    }
  }
}

void turn_right(){
  Motor_Right->run(BACKWARD);
    while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
      delay(10);
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
}

void turn_left(){
  Motor_Right->run(BACKWARD);
    while(extreme_right != 0 && right != 1 && left != 1 && extreme_left != 0){
      delay(10);
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
}