#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;
int input_pin = 11;


//int blue_led_pin;
//int green_led_pin;

int left_motor_pin = 2;
int right_motor_pin = 3;

bool isMoving = false;
int count_split = 0; //Checks Split Count
int right_count = 0; //Checks number of juncitons where right turn could be taken
int left_count = 0; //checks number of junctions where left turn could be taken 
int end = 0;

unsigned long debounceDelay = 80; // 100 ms debounce delay
unsigned long rightbounceDelay = 90; //
unsigned long leftbounceDelay = 90;
unsigned long rightDebounceTime = 0; // Last time the right condition was detected
unsigned long leftDebounceTime = 0;  // Last time the left condition was detected
unsigned long splitDebounceTime = 0; // Last time the split condition was detected


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
  pinMode(input_pin, INPUT); 
}

void loop() {
  static bool motorsRunning = true; // Track the state of the motors
  static unsigned long lastDebounceTime = 0; // For debouncing
  const unsigned long debounceDelay = 50; // Debounce delay

  int pin_val = digitalRead(input_pin);

  // Check if button state has changed
  if (pin_val == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis(); // Update the last debounce time

    // Toggle motor state
    motorsRunning = !motorsRunning;

    if (motorsRunning) {
      move(); // Start moving
    } else {
      Motor_Right->run(RELEASE); // Stop motors
      Motor_Left->run(RELEASE);
    }
  }

  // If motors are running, continue executing move logic
  if (motorsRunning) {
    move();
  }
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
    if(!isMoving){
    isMoving = true;
    }
    //Serial.println("Move Forward");
  }
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}      
  else if(extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
    if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
      rightDebounceTime = millis(); // Update debounce time
      right_count++;
      Serial.print("Right Count ");
      Serial.println(right_count);
      if (right_count == 2 || right_count == 3 || right_count == 4 || right_count == 5) {
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
  }
  
  else {
    rightDebounceTime = millis(); // Reset debounce time if condition is no longer met
  }

  // Debounce logic for left turn
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
    if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
      leftDebounceTime = millis(); // Update debounce time
      left_count++;
      Serial.println(left_count);
      if (left_count == 1) {
        turn_left(extreme_right, right, left, extreme_left);
      }
      // Add turn_left logic if needed for specific counts
    }
  }
  else {
    leftDebounceTime = millis(); // Reset debounce time if condition is no longer met
  }

  // Debounce logic for split
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
    if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
      splitDebounceTime = millis(); // Update debounce time
      count_split++;
      Serial.print("Split Count ");
      Serial.println(count_split);
      if (count_split == 1 || count_split == 3 || count_split == 4) {
        turn_left(extreme_right, right, left, extreme_left);
      }
      else if (count_split == 2 || count_split == 5) {
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
  }
  else {
    splitDebounceTime = millis(); // Reset debounce time if condition is no longer met
  }
  
  /*else if(extreme_right==1 && right == 1 && left == 1 && extreme_left == 1){
    count_split = count_split + 1;
    Serial.print("Split Count ");
    Serial.println(count_split);
    if(count_split == 1){turn_left(extreme_right, right, left, extreme_left);}
    else if(count_split == 2){turn_right(extreme_right, right, left, extreme_left);}
  }
  else if(extreme_right == 0 && right == 1 && left == 1 && extreme_left ==1){
    delay(100);
    Serial.println(left_count);
    left_count = left_count + 1;
  }
  else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left ==0){ //Turn Right
    delay(100);
    //Serial.println(right_count);
    right_count = right_count + 1;
    Serial.print("Right Count ");
    Serial.println(right_count);
    if (right_count == 2){turn_right(extreme_right, right, left, extreme_left);}
    else if(right_count == 3){turn_right(extreme_right, right, left, extreme_left);}
    else if(right_count == 4){turn_right(extreme_right, right, left, extreme_left);}
  }
  */
  if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
    if (isMoving){
    end = end + 1;
    }
    isMoving = false;
    if (end == 1){
      delay(1000);
      Motor_Right->setSpeed(200);
      Motor_Left->setSpeed(200);
      Motor_Right->run(BACKWARD);
      Motor_Left->run(BACKWARD);
      delay(200);
      isMoving = true;
      while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
        if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){ //Move backward when middle 2 sensors on the line
           Motor_Left->setSpeed(255);
           Motor_Right->setSpeed(255);
           Motor_Left->run(BACKWARD);
           Motor_Right->run(BACKWARD);
           if(!isMoving){
            isMoving = true;
          }
        }
        else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){shift_left_back(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
        else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1){shift_left_back(extreme_right, right, left, extreme_left);}
        else if(extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1){shift_left_back(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
        else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right_back(extreme_right, right, left, extreme_left);}
        else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right_back(extreme_right, right, left, extreme_left);}      
        else if(extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0){shift_right_back(extreme_right, right, left, extreme_left);}
      }
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }
  }
}

void turn_right(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  delay(500);
  Motor_Right->run(BACKWARD);
  delay(500);
  if(!isMoving){
    isMoving = true;
    }
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
  delay(500);
  if(!isMoving){
    isMoving = true;
    }
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
  Motor_Right->setSpeed(0);
  if(!isMoving){
    isMoving = true;
    }
}

void shift_right_back(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(BACKWARD);
  Motor_Right->run(BACKWARD);
  Motor_Right->setSpeed(0);
  if(!isMoving){
    isMoving = true;
    }
  /*while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
  }*/
}
void shift_left(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Left->setSpeed(0);
  if(!isMoving){
    isMoving = true;
    }
}
void shift_left_back(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(BACKWARD);
  Motor_Right->run(BACKWARD);
  Motor_Left->setSpeed(0);
  if(!isMoving){
    isMoving = true;
  }
  /*while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
    }*/
}





void pickup(){

}
