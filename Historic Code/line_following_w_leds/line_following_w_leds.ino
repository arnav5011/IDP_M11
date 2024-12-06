#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M2 and M3
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);

// Define LED pins
const int blueLED = 6;
const int redLED = 7;
const int greenLED = 8;

// Define sensor pins
const int extreme_left_pin = 2;
const int left_pin = 3;
const int right_pin = 4;
const int extreme_right_pin = 5;
const int magnetSensorPin = 9;  // Assuming magnet sensor is on pin 9

// Timing variables for flashing
unsigned long previousMillis = 0;
const long interval = 100; // 5Hz blinking (100ms on/off)

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test with LED control!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(255);
  myMotor2->setSpeed(255);

  // turn off motors
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);

  // Set up pin modes
  pinMode(extreme_left_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);
  pinMode(extreme_right_pin, INPUT);
  pinMode(magnetSensorPin, INPUT);

  // Set up LED pins
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
}

void loop() {
  int extreme_left = digitalRead(extreme_left_pin);
  int left = digitalRead(left_pin);
  int right = digitalRead(right_pin);
  int extreme_right = digitalRead(extreme_right_pin);
  
  Serial.print(extreme_left);
  Serial.print(left);
  Serial.print(right);
  Serial.println(extreme_right);
  
  if (extreme_left == 0 && extreme_right == 0 && left == 1 && right == 1) {
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    flashBlueLED(); // Flash blue LED when moving
  } else {
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    digitalWrite(blueLED, LOW); // Turn off blue LED when stopped
  }

  // Check magnet sensor and control red/green LEDs
  if (digitalRead(magnetSensorPin) == HIGH) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  } else {
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, HIGH);
  }

  delay(100);
}

void flashBlueLED() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(blueLED, !digitalRead(blueLED));
  }
}
