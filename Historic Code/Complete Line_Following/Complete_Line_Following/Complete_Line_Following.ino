#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;

int blue_led_pin = 6;
int green_led_pin = 7;

int button = 8

int left_motor_pin = 2;
int right_motor_pin = 3;



bool isMoving = false;
bool start = false

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
  pinMode(button, INPUT_PULLUP); // Use internal pull-up resistor

  pinMode(blue_led_pin, OUTPUT);
  pinMode(green_led_pin, OUTPUT);

}

void loop() {
  if (digitalRead(button) == LOW) { // Button pressed (active LOW)
    start = !start; // Toggle start state
    delay(200);     // Debounce delay
  }
  if (start) {
    Serial.println("Motors are running...");
    driveMotors();
  } else {
    Serial.println("Motors are stopped.");
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
    isMoving = false;
  }
  digitalWrite(blue_led_pin, start ? HIGH : LOW);
}

void driveMotors() {
  int extreme_right = digitalRead(extreme_right_pin);
  int right = digitalRead(right_pin);
  int left = digitalRead(left_pin);
  int extreme_left = digitalRead(extreme_left_pin);

  // Main driving logic
  while (start) { // Ensure we exit if the button is pressed again
    extreme_right = digitalRead(extreme_right_pin);
    right = digitalRead(right_pin);
    left = digitalRead(left_pin);
    extreme_left = digitalRead(extreme_left_pin);

    if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0) {
      // Moving straight
      Serial.println("Moving Forward");
      Motor_Left->run(FORWARD);
      Motor_Right->run(FORWARD);
      Motor_Left->setSpeed(255);
      Motor_Right->setSpeed(255);
    } 
    else if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) {
      // Turn right until back to 0110
      Serial.println("Turning Right");
      Motor_Left->run(FORWARD);
      Motor_Right->run(FORWARD);
      delay(100); // Move forward briefly
      Motor_Right->run(BACKWARD); // Start turning right

      while (!(digitalRead(extreme_right_pin) == 0 && digitalRead(right_pin) == 1 && digitalRead(left_pin) == 1 && digitalRead(extreme_left_pin) == 0)) {
            if (digitalRead(button) == LOW) { // Button pressed (active LOW)
              start = !start; // Toggle start state
              delay(200);     // Debounce delay
            } 
            if (start) return; // Exit if button is pressed again
            delay(10);
      }
    Motor_Right->run(FORWARD); // Resume forward motion
    } 
    else if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) {
      // Turn left until back to 0110
      Serial.println("Turning Left");
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
      delay(100); // Move forward briefly
      Motor_Left->run(BACKWARD); // Start turning left
      while (!(digitalRead(extreme_right_pin) == 0 && digitalRead(right_pin) == 1 && digitalRead(left_pin) == 1 && digitalRead(extreme_left_pin) == 0)) {
            if (digitalRead(button) == LOW) { // Button pressed (active LOW)
              start = !start; // Toggle start state
              delay(200);     // Debounce delay
            } 
            if (start) return; // Exit if button is pressed again
            delay(10);
      }
      Motor_Left->run(FORWARD); // Resume forward motion
    } 
    else if((extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0)){
      // Move to the right slightly
      Serial.println("Shifting to right");
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
      Motor_Right->setSpeed(200);
      while (!(digitalRead(extreme_right_pin) == 0 && digitalRead(right_pin) == 1 && digitalRead(left_pin) == 1 && digitalRead(extreme_left_pin) == 0)) {
            if (digitalRead(button) == LOW) { // Button pressed (active LOW)
              start = !start; // Toggle start state
              delay(200);     // Debounce delay
            } 
            if (start) return; // Exit if button is pressed again
            delay(10);
      }
      Motor_Right->setSpeed(250);
    }
    else if((extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0)){
      // Move to the right slightly
      Serial.println("Shifting to left");
      Motor_Right->run(FORWARD);
      Motor_Left->run(FORWARD);
      Motor_Left->setSpeed(200);
      while (!(digitalRead(extreme_right_pin) == 0 && digitalRead(right_pin) == 1 && digitalRead(left_pin) == 1 && digitalRead(extreme_left_pin) == 0)) {
            if (digitalRead(button) == LOW) { // Button pressed (active LOW)
              start = !start; // Toggle start state
              delay(200);     // Debounce delay
            } 
            if (start) return; // Exit if button is pressed again
            delay(10);
      }
      Motor_Left->setSpeed(250);
    }
      // Stop motors if no valid path is detected
      Serial.println("Stopping");
      Motor_Left->run(RELEASE);
      Motor_Right->run(RELEASE);
      isMoving = false;
    }

    if (!start) break; // Exit loop if button is pressed again
  }
} 
/*
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
*/