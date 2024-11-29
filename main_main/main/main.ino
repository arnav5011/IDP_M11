#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Initialize Pins for line sensors 
int extreme_right_pin = 2; 
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

int input_pin = 11; //Pin for button

int blue_led_pin = 6;  // Blue LED (motion)
int red_led_pin = 7;   // Red LED (magnetic object)
int green_led_pin = 8; // Green LED (non-magnetic object)

int time_of_flight; // Time of flight sensor.
int mag_sensor_1; // Magnetic Sensor Pin
int mag_sensor_2;

int left_motor_pin = 2;
int right_motor_pin = 3;

bool isMoving = false; //Define state of motion to handle flickering of blue LED
bool tof_active = true; //Define whether to use readings from time of flight
unsigned long tof_threshold; //Define threshold from which obstacles should be picked up
bool object_detected = false;
int object_count;
bool collected;
bool isMagnetic;

//Check number of splits, right junctions, left junctions, to work around path taken
int count_split = -1; //Initialize -1 considering start conditions.
int right_count = 0; //Checks number of juncitons where right turn could be taken
int left_count = 0; //checks number of junctions where left turn could be taken 
int end = 0; //checks number of ends reached

unsigned long debounceDelay = 80; // Minimum time sensors detect as split junction. Helps to differentiate between split or right/left junction
unsigned long rightbounceDelay = 90; // Minimum time sensors detect right junction. Helps to differentiate between split or right/left junction
unsigned long leftbounceDelay = 90; // Minimum time sensors detect left junction. Helps to differentiate between split or right/left junction

unsigned long rightDebounceTime = 0; // Last time the right condition was detected
unsigned long leftDebounceTime = 0;  // Last time the left condition was detected
unsigned long splitDebounceTime = 0; // Last time the split condition was detected


Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
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

  pinMode(input_pin, INPUT); 

  pinMode(blue_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);
  
  
  pinMode(mag_sensor_1, INPUT);
  pinMode(mag_sensor_2, INPUT);

}

void loop() {
  static bool motorsRunning = false; // Track the state of the motors
  static int lastButtonState = LOW; // Store the last button state
  int currentButtonState = digitalRead(input_pin); // Read the current button state

  if (currentButtonState == HIGH && lastButtonState == LOW) {
    delay(debounceDelay); // Simple debounce delay
    motorsRunning = !motorsRunning; // Toggle motor state

    if (motorsRunning) {
      run(); // Start running
    } else {
      Motor_Right->run(RELEASE); // Stop motors
      Motor_Left->run(RELEASE);
      digitalWrite(blue_led_pin, LOW); // Turn off blue LED
    }

    if (motorsRunning) {
    run();
  }

  handleLEDs();
  }

  lastButtonState = currentButtonState;

}

void run() {
  int extreme_right = digitalRead(extreme_right_pin); 
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);
  
  path_follow(extreme_right, right, left, extreme_left);

  if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
    if (end == 0){
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(255);
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
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
  }

  if (object_count == 1 && isMagnetic && collected) {
    while(collected) {
      int extreme_right = digitalRead(extreme_right_pin); 
      int right = digitalRead(right_pin);
      int left = digitalRead(left_pin);
      int extreme_left = digitalRead(extreme_left_pin);
      
      path_follow(extreme_right, right, left, extreme_left);
      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) { // Possible right turn
        if (millis() - rightDebounceTime >= rightbounceDelay) { // Check if condition lasts for debounceDelay
          rightDebounceTime = millis(); // Update debounce time
          right_count++;
          Serial.print("Right Count ");
          Serial.println(right_count);
        if (right_count == 2 || right_count == 4) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {rightDebounceTime = millis();} // Reset debounce time if condition is no longer met

      // Debounce logic for left turn
      if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
        if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
          leftDebounceTime = millis(); // Update debounce time
          left_count++;
          Serial.println(left_count);
          if (left_count == 1 || left_count == 3 || left == 4) {turn_left(extreme_right, right, left, extreme_left);}
        // Add turn_left logic if needed for specific counts
        }
      }
      else {leftDebounceTime = millis();} // Reset debounce time if condition is no longer met
    
    // Debounce logic for split
      if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) { // Possible split
        if (millis() - splitDebounceTime >= debounceDelay) { // Check if condition lasts for debounceDelay
          splitDebounceTime = millis(); // Update debounce time
          count_split++;
          Serial.print("Split Count ");
          Serial.println(count_split);
          if (count_split == 1) {turn_left(extreme_right, right, left, extreme_left);}
          else if (count_split == 2 || count_split == 3) {turn_right(extreme_right, right, left, extreme_left);}
        }
      }
      else {splitDebounceTime = millis();} // Reset debounce time if condition is no longer met
      if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
        Motor_Right->run(RELEASE);
        Motor_Left->run(RELEASE);
        if (end == 0){
          Motor_Right->setSpeed(255);
          Motor_Left->setSpeed(255);
          Motor_Right->run(FORWARD);
          Motor_Left->run(FORWARD);
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
      }
    }
  }
  
}

void path_follow(int extreme_right, int right, int left, int extreme_left){
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0){ //Move forward when middle 2 sensors on the line
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(FORWARD);
    Motor_Right->run(FORWARD);
    if(!isMoving){isMoving = true;}
  }
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1){shift_left(extreme_right, right, left, extreme_left);} //Off coursed to rightside so turn to left
  
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
  else if(extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0){shift_right(extreme_right, right, left, extreme_left);}
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
  Motor_Right->setSpeed(100);
  if(!isMoving){
    isMoving = true;
  }
  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0))) {
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
  }
}


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
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
  }
}

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

void detect_object() {

}

void collect_object() {

}

void depoit_object() {

}

void check_mag(){

}