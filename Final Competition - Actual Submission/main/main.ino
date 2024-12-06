/*The code uploaded to run in the final competition was intended to pick up 1 object.
Unfortunately we were not able to test this. The parts of picking up the object have been commented out.
So ideally the robot should be able to move from the start to the recycling spot and then back. */

#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Pin number for line sensors
int extreme_right_pin = 2; 
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

//Pin number for button
int button_pin = 11;

//Pins for Magnetic Sensor
int mag_sensor_1 = 9;
int mag_sensor_2 = 10;

//LED Pins
int blue_led_pin = 6;  //Blue LED (motion)
int red_led_pin = 7;   //Red LED (magnetic object)
int green_led_pin = 8; //Green LED (non-magnetic object)

//Pin numbers for motors
int left_motor_pin = 2;
int right_motor_pin = 1;

//Servo criteria
int SERVO_PIN = 12;
const int THRESHOLD = 90; //Threshold for Time of Flight for the servo to turn
const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 85; //Angle the motor rotates by

//Initializing current states
bool isMoving = false; //Is the AGV moving?
bool object_detected = false; //Is an object detected
bool isMagnetic = false; //Is the object magnetic
bool isActivated = true; //Activates time of flight sensor so that the readings are accurate
bool isRotating = false; //Is servo rotating
int currentPosition = DEFAULT_POSITION;


//Initialize Counts to help AGV navigate. Make decisions based on the number of type of nodes it as
int count_split = -1; //Set it as -1 as we start it at from the black box region and when it moves forward it detects a count. Since no action is taken at the 
int right_count = 0; //Checks number of juncitons where right turn could be taken
int left_count = 0; //Checks number of junctions where left turn could be taken 
int end = 0; //Checks the number of times reaches a black space. If we were running the code with the pick up it would have been initialized as -1

//Implementing Debounce to differentiate between split junctions and right or left Junctions.
//Longer times for right and left so that its detects a split first and then right or left.
unsigned long debounceDelay = 80; //Used this as delay for the button as well as split junctions
unsigned long rightbounceDelay = 90;
unsigned long leftbounceDelay = 90;  
unsigned long rightDebounceTime = 0; //Last time the right condition was detected
unsigned long leftDebounceTime = 0;  //Last time the left condition was detected
unsigned long splitDebounceTime = 0; //Last time the split condition was detected

//Initializing the motoros, time of flight sensor, and servo
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);
DFRobot_VL53L0X sensor;
Servo myservo;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  //Set motor speeds
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);

  //Set motor states
  Motor_Left->run(RELEASE);
  Motor_Right->run(RELEASE);

  //Set pins for line sensors 
  pinMode(extreme_left_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(extreme_right_pin, INPUT);
  
  //Set pins for button
  pinMode(button_pin, INPUT);

  //Set pins for magnetic sensor
  pinMode(mag_sensor_1, INPUT);
  pinMode(mag_sensor_2, INPUT);

  //Set output pins for the LEDs
  pinMode(blue_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);

  //Set servo pins
  myservo.attach(SERVO_PIN);
  myservo.write(DEFAULT_POSITION);

  //Initialize time of flight sensor
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();
}

void loop() {
  static bool motorsRunning = false; //Track the state of the motors. This is different from isMoving as isMoving is a response to motor motion. Motors running sets the motors to run
  static int lastButtonState = LOW; //Store the last button state
  int currentButtonState = digitalRead(button_pin); // Read the current button state

  //Detect whether button pressed
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    delay(debounceDelay); //Simple debounce delay
    motorsRunning = !motorsRunning; //Toggle motor state

    if (motorsRunning) {
      move(); // Start moving
    } else {
      Motor_Right->run(RELEASE); // Stop motors
      Motor_Left->run(RELEASE);
      digitalWrite(blue_led_pin, LOW); // Turn off blue LED
    }
  }

  // Update the last button state
  lastButtonState = currentButtonState;

  handleLEDs(); //Create flashing of LEDs
}

void move(){
  //Read line sensor values constantly from each set sensor
  int extreme_right = digitalRead(extreme_right_pin); 
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);

  line_follow(extreme_right, right, left, extreme_left); //Keep following the line

  //if(!object_detected){detect_object();} We could not test the object detection and pickup in the final test.

  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { //Detect right split.
    if (millis() - rightDebounceTime >= rightbounceDelay) {//If it has detected a right split for the desired amount of time, then count as a right
      rightDebounceTime = millis(); //Update debounce time
      right_count++;
      Serial.print("Right Count ");
      Serial.println(right_count);
      //Trace out path and make the appropriate right turn.
      if (right_count == 2 || right_count == 3 || right_count == 4 || right_count == 5) {turn_right(extreme_right, right, left, extreme_left);}
    }
  }
  else {rightDebounceTime = millis();} //Reset debounce time if condition is no longer met

  //Similar function for left turns like right turns
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) {
    if (millis() - leftDebounceTime >= leftbounceDelay) {
      leftDebounceTime = millis();
      left_count++;
      Serial.println(left_count);
      if (left_count == 1) {turn_left(extreme_right, right, left, extreme_left);}
    }
  }
  else {leftDebounceTime = millis();}
  
  //Similar function for split turns except now you make left turns and right turns as well based on the count.
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
    if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
      splitDebounceTime = millis(); // Update debounce time
      count_split++;
      Serial.print("Split Count ");
      Serial.println(count_split);

      if (count_split == 1 || count_split == 3) {turn_left(extreme_right, right, left, extreme_left);}
      else if(count_split == 0) {
        Motor_Left->setSpeed(255);
        Motor_Right->setSpeed(255);
        Motor_Left->run(FORWARD);
        Motor_Right->run(FORWARD);
        delay(1000);
      }
      else if (count_split == 2 || count_split == 4) {turn_right(extreme_right, right, left, extreme_left);}
    }
  }
  else {splitDebounceTime = millis();} // Reset debounce time if condition is no longer met */
  

  //In the state where the AGV reaches a black space completely
  if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    //Stop the motors making a decision
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);

    if (isMoving){end++;} //Increment end count only if you arrive here after moving. This is to avoid cases wherein the robot still records an end while not running.
    isMoving = false;
    
    if (end == 0){ //Starting point. Had we used the object pickup the this would be end == -1
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(255);
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
      isMoving = true;
      delay(200);
      isMoving = true;
      while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }
    /*else if (end == 0) { //This would have been run had the robot picked up the objects.
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(225);
      Motor_Right->run(BACKWARD);
      Motor_Left->run(BACKWARD);
      while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }
    } */
    if (end == 1){ //Run this when it reaches the end of the  
      delay(1000);
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(225);
      Motor_Right->run(BACKWARD);
      Motor_Left->run(BACKWARD);
      isMoving = true;
      delay(250);

      //Release objects
      //Motor_Right->run(RELEASE);
      //Motor_Left->run(RELEASE);
      //deposit_object();
      //Motor_Right->run(BACKWARD);
      //Motor_Left->run(BACKWARD);
      
      while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){ //Keep going back until reaches a split
        delay(10);
        extreme_right = digitalRead(extreme_right_pin);
        right = digitalRead(right_pin);
        left = digitalRead(left_pin);
        extreme_left = digitalRead(extreme_left_pin);
      }

      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }

    
    else if (end == 2) { //When the code just reaches the starting position again, stop inside the beox.
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(255);
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);

      delay(250);

      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }
    else {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
    }
  }
}

void turn_right(int extreme_right, int right, int left, int extreme_left){
  /*Helper function to help the AGV turn right */

  //Go ahead slightly before turning.
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  delay(250);

  //Rotate wheel backward to turn.
  Motor_Right->run(BACKWARD);
  delay(500);
  
  //Keep turning untill reaches back to line
  while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
    delay(10);
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
  }

  //Reset direction to run forward
  Motor_Right->run(FORWARD);
  //delay(50);
}

void turn_left(int extreme_right, int right, int left, int extreme_left){
  /*Helper function to help the AGV turn right */

  //Go ahead slightly before turning.
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  delay(250);

  //Rotate wheel backward to turn.
  Motor_Left->run(BACKWARD);
  
  //Keep turning untill reaches back to line
  while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0)){
    delay(10);
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
  }

  //Reset direction to run forward
  Motor_Left->run(FORWARD);
  //delay(50);
}
void shift_right(int extreme_right, int right, int left, int extreme_left){

  //To shift right, slow down right motor.
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Right->setSpeed(100);

  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0 ||
         (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1))) { 
    //Keep shifiting right until reach line or split, or end. This is to stop and break from the line correction and enter other parts of the code.
                                            
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
  }
}

void shift_left(int extreme_right, int right, int left, int extreme_left){
  
  //To shift left, slow down left motor
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Left->setSpeed(100);
  
 while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0 ||
         (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1))) { 
    //Keep shifiting right until reach line or split, or end. This is to stop and break from the line correction and enter other parts of the code.
    
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
  }
}

void line_follow(int extreme_right, int right, int left, int extreme_left){
  //Helper function to ensure line following is active.

  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){ //Move forward when middle 2 sensors on the line
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    if(!isMoving){isMoving = true;}
  }

  //Off coursed to rightside so turn to left
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){shift_left(extreme_right, right, left, extreme_left);} 
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);}
  
  //Off coursed to rightside so turn to right
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}      
  else if(extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);} 
}



void handleLEDs() {
  //Keep flashing LEDs
  unsigned long currentMillis = millis();

  //Blue LED flashing logic for motion indication
  static unsigned long lastBlueLEDFlash = 0;
  static unsigned long blueLEDDuration = 500; //Flash duration (ms)
  static bool blueState = LOW;

  if (isMoving) {
    if (currentMillis - lastBlueLEDFlash >= blueLEDDuration) {
      blueState = !blueState;
      digitalWrite(blue_led_pin, blueState);
      lastBlueLEDFlash = currentMillis;
    }
  } else {
    digitalWrite(blue_led_pin, LOW); // Turn off blue LED if not moving
  }
}

void detect_object(){
  //Using time of flight sensor to detect object. Only detect object if the time of flight is active and the servo is not rotating
  if (isActivated && !isRotating) {
    int distance = sensor.getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);
  
    if (currentPosition == DEFAULT_POSITION && distance < THRESHOLD) { //Close the servo only if the servo is in an open state and an object is detected.
      delay(500);
      moveServo(ACTIVATED_POSITION);
      isActivated = false;
      object_detected = true;
      check_magnetism() //Check the magnetism of the collected object
    }
  }
}

void moveServo(int targetPosition) {
  //Helper function to rotate the servo.

   int step; //Step size for motor rotation
   
   delay(100);
   //Stop motors when picking up object.
   Motor_Right->run(RELEASE);
   Motor_Left->run(RELEASE);


   isRotating = true; //Prevent interruptions during movement since it starts rotating.

   //Creating steps for fine-grained steps for smooth rotation
   if(targetPosition>currentPosition){step = 1;}
   else{step = -1;}
  
   while (currentPosition != targetPosition) {
       currentPosition += step;

       //Constrain the servo position
       currentPosition = constrain(currentPosition, DEFAULT_POSITION, 180);
       myservo.write(currentPosition);
       delay(10); //Adjust speed of movement by modifying this delay
   }
   isRotating = false; //Movement completed
}

void deposit_object(){
  moveServo(0); //Deposit the object 
}

void check_magnetism(){

  //Check whether the object is magnetic or not and display the appropriate light.
  if (digitalRead(mag_sensor_1) == HIGH || digitalRead(mag_sensor_2) == HGIH) {
    digitalWrite(red_led_pin, HIGH);
    digitalWrite(green_led_pin, LOW);
    isMagnetic = true;
  }
  else{
    digitalWrite(green_led_pin, HIGH);
    digitalWrite(red_led_pin, LOW);
    isMagnetic = false;
  }
}
