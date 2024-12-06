/*This was the final code written to run the AGV but we never had a chance to use it.
The code had a lot of redundancies and was not bug tested. There was not enough time to fix it and thorouhly run it.
Thus it was not used. However, the code used for the final competition was based off it. */

/*The navigation was done based on keeping the counts of the junctions. Based on the path, the appropraite navigational decision was made.

/*Rather than having multiple if else statements, it made more sense that irrespective of the object being magnetic or not, the AGV goes to common node. After reaching the
common node, its navigation could be controlled using "common" junctions. This would not yield in the most effective path, but help in simplifying path. For the first path, 
the top left node (with respect to task statement sheet) was used. For the other objects, the top right node was used*/

/*To help read and follow through the code, it is a good assumption to take each path function as a black box whose sole purpose is to move the AGV from the object to the appropriate node.
After reaching the node, the navigation is then controlled by the common junctions in the run function. It is important to note that it may seem that these variables are not incremented;
however, they are. The reason the actions at each split are spread out is to ensure better readablity, considering the structure of the run function follows the complete path of the AGV*/

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
int THRESHOLD = 90; // Threshold for Time of Flight for the servo to turn
int DEFAULT_POSITION = 0;
int ACTIVATED_POSITION = 85; //Angle the motor rotates by

//Initializing current states
bool isMoving = false; //Is the AGV moving?
bool object_detected = false; //Is an object detected
bool isMagnetic = false; //Is the object magnetic
bool collected = false; //Is the object collected
bool isActivated = true; //Activates time of flight sensor so that the readings are accurate
bool isRotating = false; //Is servo rotating
int object_count = 0;
int currentPosition = DEFAULT_POSITION; //Servo unrotated at start
int extreme_right; //Reading from line sensor on the extreme right (4th line sensor on robot when viewed frop top and facing the front)
int right; //Reading from line sensor on the closer right (3rd line sensor on robot when viewed frop top and facing the front)
int left; //Reading from line sensor on the closer left (2nd line sensor on robot when viewed frop top and facing the front)
int extreme_left; //Reading from line sensor on the closer left (1st line sensor on robot when viewed frop top and facing the front)

//Initialize Counts to help AGV navigate. Make decisions based on the number of type of nodes it as
int split_count = 0; //Checks number of juncitons where split junction occurs. This controls movement of the AGV after it picks up and object and till it reaches the common node. 
int right_count = 0; //Checks number of juncitons where right turn could be taken. This controls movement of the AGV after it picks up and object and till it reaches the common node.
int left_count = 0; //Checks number of junctions where left turn could be taken. This controls movement of the AGV after it picks up and object and till it reaches the common node.
//The counts above are reset after reaching the common node

int end = 0; //Demarkate region of no line (start/end)

int common_right = 0; //Controls AGV movement untill a box is found.
int common_left = 0; //Controls AGV movement untill a box is found.
int common_split = 0; //Controls AGV movement untill a box is found.
/*The counts above have a similar functioning as to the split_count, right_count, left_count. These counts are used for controlling the AGV when there is no object detected.
This was done because when there is no object detected, it does not enter any path function.
For this specific reason, these counts are not resetted anytime.*/

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
      run(); // Start moving
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

void run() {

  //Read line sensor values constantly from each set sensor 
  read_line_sensors();
  line_follow(extreme_right, right, left, extreme_left);

  //Starting from the box. Move forward from there.
  if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
    if (end == 0){
      end = end + 1;
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(255);
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
      delay(200);
      isMoving = true;
      while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
        delay(10);
        read_line_sensors();
      }
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }
  }

  //Keep detecting objects untill already picked one up.
  if(!object_detected){detect_object();}

  //Collected first object at the first split. Trace path to respective station.
  path_object_1();

  //At this point, the AGV should be at the top left corner. Hence we could use the common_left, common_right, common_split, and the main loop to control navigation 
  
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
    if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
      leftDebounceTime = millis(); // Update debounce time
      common_left++;
      Serial.print("Common Left ");
      Serial.println(common_left);
      if (common_left == 1) {
        turn_left(extreme_right, right, left, extreme_left);
        servo_open(); //Open the servo since we know the object is near
      }
    }
  }
  else {leftDebounceTime = millis();}
  //After some point the object should be detected (the detect function is running within the main loop)
  
  //The paths for the case of having collected 2 objects will be traced.
  path_object_2();

  //Once again we can use the common parameters to help the AGV naviagate to the next object.
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
    if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
      rightDebounceTime = millis(); // Update debounce time
      common_right++;
      if (common_right == 1) {
        turn_right(extreme_right, right, left, extreme_left);
        servo_open();
      }
    }
  }
  else {rightDebounceTime = millis();} // Reset debounce time if condition is no longer met
  
  //Once the third object is collected enter path for the third object
  path_object_3();
  

  /*Ideally all the common right, left, and split turns should be in one if block. However, for code clarity and making it easy to follow the paths, it made sense to have more if blocks
  for the same right/left turns. Within these blocks, the counts of the common right and left would not be updated. They would be updated in the blocks above.*/

  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
    if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
      rightDebounceTime = millis(); // Update debounce time
      Serial.println(common_right);
      //1 common right executed before for the case going to the third box, 2nd common right ignored, 3rd common right takes to 4th box
      if(common_right == 3) {turn_right(extreme_right, right, left, extreme_left);} 
    }
  }
  else {leftDebounceTime = millis();}
  
  //The AGV should have detected the fourth object.
  path_object_4();

  //Back to the top right corner. Now we navigate back to the start.
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
      if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
        rightDebounceTime = millis(); // Update debounce time
        if(common_right == 5) {turn_right;}
      }
    }
  else {rightDebounceTime = millis();}

  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
      if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
        leftDebounceTime = millis(); // Update debounce time
        if(common_left==3) {turn_left(extreme_right, right, left, extreme_left);}
      }
    }
  else {leftDebounceTime = millis();}

  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
    if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
      splitDebounceTime = millis(); // Update debounce time
      common_split++;
      if (common_split == 1) {
        delay(2500);
        Motor_Right->run(RELEASE); //Stop the AGV at end
        Motor_Left->run(RELEASE);
      }
    }
  }
  else {splitDebounceTime = millis();} 

}
  
//Functions to help move the AGV:

//Line_follow ensures the AGV remains on the line. In the case of the AGV going offcourse, it corrects it back to go straight along the white line by changing motor speeds
void line_follow(int extreme_right, int right, int left, int extreme_left){
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
  
  //Off coursed to leftside so turn to right
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);} 
}

//Turns right by going slightly forward and then rotating the right wheel backward. It keeps turning untill reaches the white line again. 
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
    read_line_sensors();
  }
  Motor_Right->run(FORWARD);
  //delay(50);
}

//Turns left by going slightly forward and then rotating the left wheel backward. It keeps turning untill reaches the white line again. 
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
    read_line_sensors();
  }
  Motor_Left->run(FORWARD);
  //delay(50);
}

//Shifts rightward when the AGV goes off course leftward by reducing speed of the right motor. It keeps shifting until it straightens.
void shift_right(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Right->setSpeed(100);
  if(!isMoving){
    isMoving = true;
  }
  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0))) {
      read_line_sensors();
  }
}

//Shifts leftward when the AGV goes off course righward by reducing speed of the left motor. It keeps shifting until it straightens.
void shift_left(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Left->setSpeed(100);
  if(!isMoving){
    isMoving = true;
  }
  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0))) {
    read_line_sensors();;
  }
}

//Functions to control servo actions

//This function detects and picks up an object and updates the states when an object is picked up. 
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
      collected = true;
      object_count++;
      check_magnetism(); //Check the magnetism of the collected object
    }
  }
}

//This function rotates the servo untill it reaches the desired position.
void moveServo(int targetPosition) {

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

//This function opens the servo and returns it to the default position. It is particularly useful in depositing objects and reopening the servo to detect objects
void servo_open(){
  moveServo(0);
}

//Rotates the servo to enclose the object. Useful in picking up objects and giving enough space for turning.
void servo_close(){
  moveServo(ACTIVATED_POSITION);
}

/*This function is a mix of navigation and motor settings. The method employed for depositing was to run the AGV backward and then stop after 250ms.
When the motors are stopped the servo opens allowing to deposit the objects at the respective site. After depositing the states are updated accordingly.
Then the AGV goes back untill it reaches a split and then the appropriate decision for turnign is made. */
void deposit(){
  delay(1000);

  //Varying Motor Speeds to ensure goes back roughly straight
  Motor_Right->setSpeed(255);
  Motor_Left->setSpeed(225);
  Motor_Right->run(BACKWARD);
  Motor_Left->run(BACKWARD);
  isMoving = true;
  delay(250);

  //Release objects
  Motor_Right->run(RELEASE);
  Motor_Left->run(RELEASE);
  servo_open();
  digitalWrite(green_led_pin, LOW);
  digitalWrite(red_led_pin, LOW);
  object_detected = false;
  collected = false;
  isActivated = true;
  
  //Run back again untill reach the split.
  delay(100);
  Motor_Right->run(BACKWARD);
  Motor_Left->run(BACKWARD);
      
  while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){ //Keep going back until reaches a split
    delay(10);
    read_line_sensors();
  }

  //Stop at split.
  Motor_Right->run(RELEASE);
  Motor_Left->run(RELEASE);
}


//Tracing paths

/*The main idea of navigation was to move from the object to the respective site and then to a common node. This was done so that irrespective
of the object magnetic or not, it would reach the same point. This approach although inefficient, greatly helped simplifying hardcoding the path.
After reaching the common node, the code breaks out of the path function and the navigation is controlled by the common left, common right and common split
counts.*/

/*The first path for the case of 1 collected object is traced from object to the respective site and then to the top left node.
The navigation is done by counting the number of instances it detects a right/left/split turn which are reset at the end of it. */
void path_object_1(){
  if (object_count == 1 && isMagnetic && collected) {
    while(collected){ //Keep running the code till reached recycling zone and deposits and goes back to split
      handleLEDs();
      //Reread sensor readings till no longer deposited.
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left); //Keep linefollowing

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

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_left(extreme_right, right, left, extreme_left);}
          else if (split_count == 2) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();} 

      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    //Reset counts to 0 for easier navigation to the desired node
    left_count = 0;
    right_count = 0;
    split_count = 0;

    /*After depositing, we reach back to the split junction. Rather than going back to the main loop function, it was chosen to stay within this if statement.
    The idea was that we go back to the main loop function when we reach the common node. */
    turn_right(extreme_right, right, left, extreme_left); //Considering back at split

    servo_close(); //Close the servo so that there is enough space for turning.

    while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
    }
    turn_left(extreme_right, right, left, extreme_left); //Turn left to face the top left most corner.
    
    while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1)){ //Keep moving untill reach the corner
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
    }
    turn_left(extreme_right, right, left, extreme_left);
    
    //Reset Counts for turning to aid in navigation for other boxes.
    left_count = 0;
    right_count = 0;
    split_count = 0;
  }


  else if (object_count == 1 && !isMagnetic && collected) {
    while(collected){ //Keep running the code till reached landfill zone and deposits and goes back to split
      handleLEDs();
      //Reread sensor readings till no longer deposited.
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left); //Keep linefollowing

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { //Detect right split.
        if (millis() - rightDebounceTime >= rightbounceDelay) {//If it has detected a right split for the desired amount of time, then count as a right
          rightDebounceTime = millis(); //Update debounce time
          right_count++;
          Serial.print("Right Count ");
          Serial.println(right_count);
          //Trace out path and make the appropriate right turn.
          if (right_count == 2 || right_count == 4) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {rightDebounceTime = millis();} //Reset debounce time if condition is no longer met

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_left(extreme_right, right, left, extreme_left);}
          else if (split_count == 2) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();} 

      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    //Reset counts to 0 for easier navigation to the desired node
    left_count = 0;
    right_count = 0;
    split_count = 0;
    /*After depositing, we reach back to the split junction. Rather than going back to the main loop function, it was chosen to stay within this if statement.
    The idea was that we go back to the main loop function when we reach the common node. */
    turn_right(extreme_right, right, left, extreme_left); //Turn right to face the top left corner considering we reached the split

    servo_close(); //Close the servo so that there is enough space for turning.

    while(!(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) && left_count<2){ //Avoid the left turn that would take it to recycling path
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
      if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
        if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
          leftDebounceTime = millis(); // Update debounce time
          left_count++;
        }
      }
      else {leftDebounceTime = millis();}
    }
    turn_left(extreme_right, right, left, extreme_left); //Turn left at top left most corner.  
    //Reset Counts for turning to aid in navigation for other boxes.
    left_count = 0;
    right_count = 0;
    split_count = 0;
  }
}

//This function traces the path of the second object to the respective site and then to the top right node.
void path_object_2() {
  if (object_count == 2 && isMagnetic && collected) {
    while(collected){
      handleLEDs();
      //Reread sensor readings till no longer deposited.
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);

      if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
        if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          left_count++;
          Serial.print("Left ");
          Serial.println(left_count);
          if (left_count == 1 ||left_count == 2) {turn_left(extreme_right, right, left, extreme_left);} //Take two lefts after picking up object from start
        }
      }
      else {leftDebounceTime = millis();}
      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    //Reset counts to 0 for easier navigation to the desired node
    return_to_top_right_recycle();
  }
  
  else if(object_count == 2 && !isMagnetic && collected) {
    while(collected){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);

      if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
        if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
          leftDebounceTime = millis(); // Update debounce time
          left_count++;
          Serial.print("Left ");
          Serial.println(left_count);
          if (left_count == 1) {turn_left(extreme_right, right, left, extreme_left);}
        }
      }
      else {leftDebounceTime = millis();}

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
        if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          right_count++;
          Serial.print("Right ");
          Serial.println(right_count);
          if (right_count == 1) {turn_right(extreme_right, right, left, extreme_left);} //Take two lefts after picking up object from start
        }
      }
      else {leftDebounceTime = millis();}

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();} 

      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    return_to_top_right_landfill();
  }
}

//This function traces the path of the third object to the respective site and then to the top right node.
void path_object_3(){
  if (object_count == 3 && isMagnetic && collected) {
    while(collected) {
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
      
      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
        if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          right_count++;
          Serial.println(right_count);
          if (right_count== 1) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {rightDebounceTime = millis();} // Reset debounce time if condition is no longer met
      
      if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
        if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
          leftDebounceTime = millis(); // Update debounce time
          left_count++;
          Serial.println(left_count);
          if (left_count== 1) {turn_left(extreme_right, right, left, extreme_left);}
        }
      }
      else {leftDebounceTime = millis();} // Reset debounce time if condition is no longer met
      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    return_to_top_right_recycle();
  }
  
  else if(object_count == 3 && !isMagnetic && collected) {
    while(collected) {
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
      
      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
        if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          right_count++;
          Serial.println(right_count);
          if (right_count== 1 || right_count == 2) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {rightDebounceTime = millis();}

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();}
      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      } 
    }
    //Reset counts to 0 for easier navigation to the desired node
    return_to_top_right_landfill();
  }
}

//This function traces the path of the fourth object to the respective site and then to the top right node.
void path_object_4(){
  if(object_count == 4 && isMagnetic && collected) {
    while(collected){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();}

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
      else {rightDebounceTime = millis();}

      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      }
    }
    return_to_top_right_recycle();
  }
  
  else if(object_count == 4 && !isMagnetic && collected){
    while(collected){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          split_count++;
          Serial.print("Split Count ");
          Serial.println(split_count);
          if (split_count == 1) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();}

      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { //Detect right split.
        if (millis() - rightDebounceTime >= rightbounceDelay) {//If it has detected a right split for the desired amount of time, then count as a right
          rightDebounceTime = millis(); //Update debounce time
          right_count++;
          Serial.print("Right Count ");
          Serial.println(right_count);
          //Trace out path and make the appropriate right turn.
          if (right_count == 2 || right_count == 4) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {rightDebounceTime = millis();}
      if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
        deposit();
      } 
    }
    return_to_top_right_landfill();
  }
} 

//Helper functions

//Blue LEDs flashing
void handleLEDs(){
  unsigned long currentMillis = millis();

  // Blue LED flashing logic for motion indication
  static unsigned long lastBlueLEDFlash = 0;
  static unsigned long blueLEDDuration = 500; // Flash duration (ms)
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

//Checking Magnetism
void check_magnetism(){

  //Check whether the object is magnetic or not and display the appropriate light.
  if (digitalRead(mag_sensor_1) == HIGH || digitalRead(mag_sensor_2) == HIGH) {
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
//Reading Line sensors
void read_line_sensors(){
  int extreme_right = digitalRead(extreme_right_pin); 
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);
}
//Since we returned to the top right corner thrice, the following are helper function to navigate to that node from their respective site.
void return_to_top_right_recycle(){
  //Reset counts to 0 for easier navigation to the desired node
    left_count = 0;
    right_count = 0;
    split_count = 0;
    turn_right(extreme_right, right, left, extreme_left); //Considering back at split

    servo_close(); //Close the servo so that there is enough space for turning.

    while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
    }
    turn_right(extreme_right, right, left, extreme_left); //Turn right to face the top right most corner.
    
    while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) && right_count<2){ //Avoid the right turn that would take it to landfill path
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left);
      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
        if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          right_count++;
        }
      }
      else {leftDebounceTime = millis();}
    }
    turn_right(extreme_right, right, left, extreme_left); //Turn right at top right most corner.  
    //Reset Counts for turning to aid in navigation for other boxes.
    left_count = 0;
    right_count = 0;
    split_count = 0;

}

void return_to_top_right_landfill(){
  //Reset counts to 0 for easier navigation to the desired node
    left_count = 0;
    right_count = 0;
    split_count = 0;
    servo_close(); //Close the servo so that there is sufficient space for turning.
    turn_left(extreme_right, right, left, extreme_left); //Face top right corner 
    while(!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0)){
      handleLEDs();
      read_line_sensors();
      line_follow(extreme_right, right, left, extreme_left); 
    }
    //while loop ends when it reaches top right corner
    turn_right(extreme_right, right, left, extreme_left);

    //Reset Counts for turning to aid in navigation for other boxes.
    left_count = 0;
    right_count = 0;
    split_count = 0;
}
      