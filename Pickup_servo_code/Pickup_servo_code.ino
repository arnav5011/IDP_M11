#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

DFRobot_VL53L0X sensor;
Servo myservo;

const int THRESHOLD = 90; // 9cm in millimetres
const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 85;
const int SERVO_PIN = 12;
const int BUTTON_PIN = 11; // Connect button to pin 11

int currentPosition = DEFAULT_POSITION;
bool isActivated = false;
bool buttonPressed = false;
bool isMoving = false; // Tracks if the servo is currently moving

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
    // Check button state
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
        buttonPressed = true; // Debounce
        if (isActivated && !isMoving) { // Ensure the servo is not already moving
            moveServo(DEFAULT_POSITION);
            isActivated = false;
        }
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonPressed = false;
    }

    // Read distance only if not activated and not moving
    if (!isActivated && !isMoving) {
        int distance = sensor.getDistance();
        Serial.print("Distance: ");
        Serial.println(distance);

        if (currentPosition == DEFAULT_POSITION && distance < THRESHOLD) {
            moveServo(ACTIVATED_POSITION);
            isActivated = true;
        }
    }

    delay(50); // Small delay to prevent excessive looping
}

void moveServo(int targetPosition) {
    isMoving = true; // Prevent interruptions during movement
    int step = (targetPosition > currentPosition) ? 1 : -1; // Fine-grained steps for smooth motion

    while (currentPosition != targetPosition) {
        currentPosition += step;

        // Constrain the servo position
        currentPosition = constrain(currentPosition, DEFAULT_POSITION, 180);
        myservo.write(currentPosition);
        delay(10); // Adjust speed of movement by modifying this delay
    }

    isMoving = false; // Movement completed
}
