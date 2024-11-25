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
move();
}

void move(){
  //Read sensor values
  int extreme_right = digitalRead(extreme_right_pin); 
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);

  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){ //Move forward when middle 2 sensors on the line
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    //Serial.println("Move Forward");
  }
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}      
  else if(extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
  else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){
    delay(100);
    Serial.println(left_count);
    left_count = left_count + 1;
  }
  else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
    delay(100);
    Serial.println(right_count);
    right_count = right_count + 1;
    if (right_count == 2){turn_right(extreme_right, right, left, extreme_left);}
    else if(right_count == 3){turn_right(extreme_right, right, left, extreme_left);}
    else if(right_count == 4){turn_right(extreme_right, right, left, extreme_left);}
  }
  else if(extreme_right==1 && right == 1 && left == 1 && extreme_left == 1){
    delay(100);
    Serial.println(count_split);
    count_split = count_split + 1;
    if(count_split == 1){turn_left(extreme_right, right, left, extreme_left);}
    else if(count_split == 2){turn_right(extreme_right, right, left, extreme_left);}
  }
  else if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
  }
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
  //delay(50);
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
  //delay(50);
}
void shift_right(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Right->setSpeed(50);
}
void shift_left(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Left->setSpeed(50);
}