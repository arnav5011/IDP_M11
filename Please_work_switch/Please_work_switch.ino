#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Existing pin definitions
int extreme_right_pin = 2;
int right_pin = 3;
int left_pin = 4;
int extreme_left_pin = 5;
int left_motor_pin = 2;
int right_motor_pin = 3;

// New button and LED pins
int ledPin = 6;    // LED pin
int buttonPin = 7; // Button pin

bool isRunning = false; // Flag to control robot movement

// Existing motor setup
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(left_motor_pin);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(right_motor_pin);

void setup() {
    Serial.begin(9600);
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");

    // Set up motors
    Motor_Left->setSpeed(255);
    Motor_Right->setSpeed(255);
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);

    // Set up sensor pins
    pinMode(extreme_left_pin, INPUT);
    pinMode(left_pin, INPUT);
    pinMode(right_pin, INPUT);
    pinMode(extreme_right_pin, INPUT);

    // Set up button and LED pins
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor
}

void loop() {
    handleButton();
    if (isRunning) {
        move();
        digitalWrite(ledPin, HIGH); // LED on when robot is running
    } else {
        stopMotors();
        digitalWrite(ledPin, LOW); // LED off when robot is stopped
    }
}

void handleButton() {
    static bool lastButtonState = HIGH;
    static unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;

    int reading = digitalRead(buttonPin);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW && lastButtonState == HIGH) {
            isRunning = !isRunning; // Toggle robot state
        }
    }

    lastButtonState = reading;
}

void stopMotors() {
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
}

// Existing move() function and other functions remain unchanged
