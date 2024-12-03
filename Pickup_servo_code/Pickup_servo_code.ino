#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

DFRobot_VL53L0X sensor;
Servo myservo;

const int THRESHOLD = 90; // 9cm in millimetres
const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 105;
const int SERVO_PIN = 12;
const int BUTTON_PIN = 11; // Connect button to pin 11

int currentPosition = DEFAULT_POSITION;
bool isActivated = false;
bool buttonPressed = false;






void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();
  
  myservo.attach(SERVO_PIN);
  myservo.write(DEFAULT_POSITION);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
}







void loop() {
  int distance = sensor.getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Check button state
  if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
    buttonPressed = true;
    if (isActivated) {
      moveServo(DEFAULT_POSITION);
      isActivated = false;
    }
  } else if (digitalRead(BUTTON_PIN) == HIGH) {
    buttonPressed = false;
  }

  // Check distance only if not activated
  if (!isActivated && distance < THRESHOLD) {
    moveServo(ACTIVATED_POSITION);
    isActivated = true;
  }

  delay(50); // Small delay to prevent excessive looping
}








void moveServo(int targetPosition) {
  int step = (targetPosition > currentPosition) ? 5 : -5;
  
  while (currentPosition != targetPosition) {
    currentPosition += step;
    if (currentPosition < 0) currentPosition = 0;
    if (currentPosition > 180) currentPosition = 180;
    myservo.write(currentPosition);
    delay(15);
  }
}
