#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

DFRobot_VL53L0X sensor;
Servo myservo;

const int THRESHOLD = 90; // 10cm in millimetres
const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 120;
const int SERVO_PIN = 12;

int currentPosition = DEFAULT_POSITION;
bool isMoving = false;

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
  int distance = sensor.getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < THRESHOLD && currentPosition != ACTIVATED_POSITION) {
    moveServo(ACTIVATED_POSITION);
  } else if (distance >= THRESHOLD && currentPosition != DEFAULT_POSITION) {
    moveServo(DEFAULT_POSITION);
  }

  delay(50); // Small delay to prevent excessive looping
}




void moveServo(int targetPosition) {
  isMoving = true;
  int step = (targetPosition > currentPosition) ? 10 : -10;
  
  while (currentPosition != targetPosition && isMoving) {
    currentPosition += step;
    myservo.write(currentPosition);
    delay(15);

    // Check if the condition has changed mid-movement
    int newDistance = sensor.getDistance();
    if ((newDistance < THRESHOLD && targetPosition == DEFAULT_POSITION) ||
        (newDistance >= THRESHOLD && targetPosition == ACTIVATED_POSITION)) {
      isMoving = false;
    }
  }
  
  isMoving = false;
}
