#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;
int button_pin = 11;
int blue_led_pin = 6;  // Blue LED (motion)
int red_led_pin = 7;   // Red LED (magnetic object)
int green_led_pin = 8; // Green LED (non-magnetic object)

int mag_sensor_1 = 9;  // Magnetic sensor 1
int mag_sensor_2 = 10; // Magnetic sensor 2



//int blue_led_pin;
//int green_led_pin;

int left_motor_pin = 2;
int right_motor_pin = 3;

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

bool isMoving = false;
int count_split = -1; //Checks Split Count
int right_count = 0; //Checks number of juncitons where right turn could be taken
int left_count = 0; //checks number of junctions where left turn could be taken 
int end = -1;

unsigned long debounceDelay = 80; // 100 ms debounce delay
unsigned long rightbounceDelay = 90; //
unsigned long leftbounceDelay = 90;
unsigned long rightDebounceTime = 0; // Last time the right condition was detected
unsigned long leftDebounceTime = 0;  // Last time the left condition was detected
unsigned long splitDebounceTime = 0; // Last time the split condition was detected


volatile bool isPaused = true;
void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  pinMode(button_pin, INPUT_PULLUP); // Use internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(button_pin), togglePause, FALLING);

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
  pinMode(button_pin, INPUT); 
  pinMode(blue_led_pin, OUTPUT);
  pinMode(red_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);

  pinMode(mag_sensor_1, INPUT);
  pinMode(mag_sensor_2, INPUT);

}

void loop() {
  if(!isPaused){move();}
  else{
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
  }
  handleLEDs();
}

void togglePause() {
  isPaused = !isPaused; // Toggle the paused state
}

void button(){
  if (isPaused){
    isMoving = false;
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
  }
  else {isMoving = true;}
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
    button();
    //if(!isMoving){isMoving = true;}
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
      if (right_count == 2 || right_count == 3 || right_count == 4 || right_count == 5) {turn_right(extreme_right, right, left, extreme_left);}
    }
  }
  else {rightDebounceTime = millis();} // Reset debounce time if condition is no longer met

  // Debounce logic for left turn
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) { // Possible left turn
    if (millis() - leftDebounceTime >= leftbounceDelay) { // Check if condition lasts for debounceDelay
      leftDebounceTime = millis(); // Update debounce time
      left_count++;
      Serial.println(left_count);
      if (left_count == 1) {turn_left(extreme_right, right, left, extreme_left);}
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
      if (count_split == 1 || count_split == 3 || count_split == 4) {turn_left(extreme_right, right, left, extreme_left);}
      else if (count_split == 2 || count_split == 5) {turn_right(extreme_right, right, left, extreme_left);}
    }
  }
  else {splitDebounceTime = millis();} // Reset debounce time if condition is no longer met
  
  if(extreme_right==0 && right == 0 && left == 0 && extreme_left == 0) {
    Motor_Right->run(RELEASE);
    Motor_Left->run(RELEASE);
    if (isMoving){end = end + 1;}
    isMoving = false;
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

    if (end == 1){
      delay(1000);
      Motor_Right->setSpeed(255);
      Motor_Left->setSpeed(225);
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
      }
      Motor_Right->run(RELEASE);
      Motor_Left->run(RELEASE);
    }
    else if (end == 2) {
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
    button();
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
    button();
  }
  Motor_Left->run(FORWARD);
  //delay(50);
}
void shift_right(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Right->setSpeed(125);
  if(!isMoving){
    isMoving = true;
  }
  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0))) {
      extreme_right = digitalRead(extreme_right_pin);
      right = digitalRead(right_pin);
      left = digitalRead(left_pin);
      extreme_left = digitalRead(extreme_left_pin);
      button();
  }
}


void shift_left(int extreme_right, int right, int left, int extreme_left){
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(FORWARD);
  Motor_Left->setSpeed(125);
  if(!isMoving){
    isMoving = true;
  }
  while (!((extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) ||
         (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0))) {
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);
    button();
  }
}

void handleLEDs() {
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

  // Read magnetic sensor values
  int mag1 = digitalRead(mag_sensor_1);
  int mag2 = digitalRead(mag_sensor_2);

  // Red and Green LED handling based on magnetic object detection
  if (mag1 == HIGH || mag2 == HIGH) {
    // Magnetic object detected
    digitalWrite(red_led_pin, HIGH);
    digitalWrite(green_led_pin, LOW);
  } else {
    // Non-magnetic object detected
    digitalWrite(red_led_pin, LOW);
    digitalWrite(green_led_pin, HIGH);
  }
}
