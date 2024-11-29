#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

DFRobot_VL53L0X sensor;
Servo myservo;

const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 120;
const int SERVO_PIN = 12;
const unsigned long SWITCH_INTERVAL = 5000; // 5 seconds in milliseconds

int currentPosition = DEFAULT_POSITION;
unsigned long lastSwitchTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();
  
  myservo.attach(SERVO_PIN);
  myservo.write(DEFAULT_POSITION);
}

void loop() {
  // Read and print distance (for debugging purposes)
  int distance = sensor.getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Check if it's time to switch positions
  unsigned long currentTime = millis();
  if (currentTime - lastSwitchTime >= SWITCH_INTERVAL) {
    // Switch position
    if (currentPosition == DEFAULT_POSITION) {
      moveServo(ACTIVATED_POSITION);
    } else {
      moveServo(DEFAULT_POSITION);
    }
    lastSwitchTime = currentTime;
  }

  delay(50); // Small delay to prevent excessive looping
}

void moveServo(int targetPosition) {
  int step = (targetPosition > currentPosition) ? 10 : -10;
  
  while (currentPosition != targetPosition) {
    currentPosition += step;
    myservo.write(currentPosition);
    delay(15);
  }
  
  Serial.print("Moved to position: ");
  Serial.println(currentPosition);
}
