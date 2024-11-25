#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

//int blue_led_pin;
//int green_led_pin;

int left_motor_pin = 2;
int right_motor_pin = 3;

int count_split = 0; //Checks Split Count
int right_count = 0; //Checks number of juncitons where right turn could be taken
int left_count = 0; //checks number of junctions where left turn could be taken 

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set motor speeds
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);

  // Set motor states
  Motor_Left->run(RELEASE);
  Motor_Right->run(RELEASE);

  pinMode(extreme_left_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(extreme_right_pin, INPUT);
  //pinMode(button, INPUT_PULLUP); // Use internal pull-up resistor

  //pinMode(blue_led_pin, OUTPUT);
  //pinMode(green_led_pin, OUTPUT);
}

void loop() {
  
  
  mov_forward_until_split_or_turn();

}

void mov_forward_until_split_or_turn(){
  int extreme_right = digitalRead(extreme_right_pin);
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);
  Serial.print(extreme_left);
  Serial.print(left);
  Serial.print(right);
  Serial.println(extreme_right);
  //while(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    Serial.println("Move Forward");
  }
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){ //Off coursed to rightside so turn to left
      Motor_Left->setSpeed(225);
      while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
        Serial.println("Shifting Left");
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
      Motor_Left->setSpeed(255);
    }
    else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){ //Off coursed to leftside so turn to right
      Motor_Right->setSpeed(225);
      while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
        Serial.println("Shifting Right");
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
      Motor_Right->setSpeed(255);
    }
    else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){ //Turn Left
      Serial.println("Turning Left");
      left_count = left_count + 1;
      if (left_count == 1){
        turn_left(extreme_right, right, left, extreme_left);
      }
    }
    else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
      right_count = right_count + 1;
      Serial.println("Turning Right");
      if (right_count == 2 || right_count == 3){
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
    else if(extreme_right==1 && right == 1 && left == 1 && extreme_left == 1) {
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      count_split = count_split + 1;
      if(count_split == 1){
        turn_left(extreme_right, right, left, extreme_left);
      }
      else if(count_split == 2){
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
    else if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0){
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
      Serial.println("STOP");
      while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
      Serial.println("STOP");
      delay(10);
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
    }
    }
  //}
}

void turn_right(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  delay(500);
  Motor_Right->run(BACKWARD);
    while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
      delay(10);
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
    }
  Motor_Right->run(FORWARD);
}

void turn_left(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  delay(500);
  Motor_Left->run(BACKWARD);
    while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
      delay(10);
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
    }
  Motor_Left->run(FORWARD);
}