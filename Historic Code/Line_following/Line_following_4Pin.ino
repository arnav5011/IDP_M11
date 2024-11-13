#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Motor objects on M1 and M2
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

// Sensor pins
const int leftPin = 4;         // Left sensor
const int centerLeftPin = 5;   // Center-left sensor
const int centerRightPin = 6;  // Center-right sensor
const int rightPin = 7;        // Right sensor

void setup() {
    Serial.begin(9600);           // Set up Serial library at 9600 bps
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    // Initialize motor shield
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");

    // Set motor speed and initialize in the stopped state
    myMotor->setSpeed(255);
    myMotor2->setSpeed(255);
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);

    // Set up sensor pins as inputs
    pinMode(leftPin, INPUT);
    pinMode(centerLeftPin, INPUT);
    pinMode(centerRightPin, INPUT);
    pinMode(rightPin, INPUT);
}

void loop() {
    // Read sensor values
    int left = digitalRead(leftPin);
    int centerLeft = digitalRead(centerLeftPin);
    int centerRight = digitalRead(centerRightPin);
    int right = digitalRead(rightPin);

    // Print sensor states for debugging
    Serial.print("Left: "); Serial.print(left);
    Serial.print(" Center Left: "); Serial.print(centerLeft);
    Serial.print(" Center Right: "); Serial.print(centerRight);
    Serial.print(" Right: "); Serial.println(right);

    // Determine movement based on sensor values
    if (left == 0 && centerLeft == 1 && centerRight == 1 && right == 0) {
        // Line is centered, move forward
        myMotor->run(FORWARD);
        myMotor2->run(FORWARD);
    } 
    else if (centerLeft == 1 && centerRight == 0) {
        // Line detected more on the left side; turn left slightly
        myMotor->run(FORWARD);
        myMotor2->run(RELEASE);
    } 
    else if (centerLeft == 0 && centerRight == 1) {
        // Line detected more on the right side; turn right slightly
        myMotor->run(RELEASE);
        myMotor2->run(FORWARD);
    } 
    else if (left == 1) {
        // Leftmost sensor detects line; make a sharper left turn
        myMotor->run(FORWARD);
        myMotor2->run(RELEASE);
    } 
    else if (right == 1) {
        // Rightmost sensor detects line; make a sharper right turn
        myMotor->run(RELEASE);
        myMotor2->run(FORWARD);
    } 
    else {
        // No line detected; stop or take corrective action
        myMotor->run(RELEASE);
        myMotor2->run(RELEASE);
    }

    delay(100);  // Short delay for stability
}
