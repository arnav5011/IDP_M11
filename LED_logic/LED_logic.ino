/*
IDP
LED Module Code for Robot with Dual Motors
*/

#include <Servo.h>
#include <Adafruit_MotorShield.h>

// Define LED pins
int blueLED = 6;
int redLED = 7;
int greenLED = 8;

// Define sensor pin
int magnetSensorPin = 2;

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);

// Servo setup
Servo myservo;
int pos = 0;

// Timing variables for flashing
unsigned long previousMillis = 0;
const long interval = 100; // 5Hz blinking (100ms on/off)

void setup() {
  Serial.begin(9600);
  Serial.println("Robot LED and Motor Control with Dual Motors");

  // Set up LEDs and sensor pin
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(magnetSensorPin, INPUT);

  // Initialize motor shield
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set initial motor speed
  myMotor1->setSpeed(255);
  myMotor2->setSpeed(255);

  // Turn off motors initially
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);

  // Attach servo
  myservo.attach(9);
}

void loop() {
  // Check if robot is moving and flash blue LED
  if (isRobotMoving()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      digitalWrite(blueLED, !digitalRead(blueLED));
    }
  } else {
    digitalWrite(blueLED, LOW);
  }

  // Check magnet sensor and control red/green LEDs
  if (digitalRead(magnetSensorPin) == HIGH) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  } else {
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
  }

  // Simulate servo movement (as in sample code)
  for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);
    delay(15);
  }
  
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }

  // Simulate motor movement similar to line-following logic
  int extreme_left = digitalRead(2);
  int left = digitalRead(3);
  int right = digitalRead(4);
  int extreme_right = digitalRead(5);

  if (extreme_left == LOW && extreme_right == LOW && left == HIGH && right == HIGH) {
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    Serial.println("Moving forward");
  } else {
    myMotor1->run(RELEASE);
    myMotor2->run(RELEASE);
    Serial.println("Stopping");
  }

  delay(100);  
}










bool isRobotMoving() {
   // Check both motors to determine if robot is moving
   return myMotor1->getState() != RELEASE || myMotor2->getState() != RELEASE;
}






