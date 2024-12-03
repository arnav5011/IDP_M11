#include <Adafruit_MotorShield.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
DFRobot_VL53L0X sensor;
Servo myservo;

// Constants
const int THRESHOLD = 90; // 9cm in millimetres
const int DEFAULT_POSITION = 0;
const int ACTIVATED_POSITION = 120;
const int SERVO_PIN = 12;
const int EXTREME_RIGHT_PIN = 2;
const int RIGHT_PIN = 3;
const int LEFT_PIN = 4;
const int EXTREME_LEFT_PIN = 5;
const int INPUT_PIN = 11; // Pin for button
const int BLUE_LED_PIN = 6; // Blue LED (motion)
const int RED_LED_PIN = 7; // Red LED (magnetic object)
const int GREEN_LED_PIN = 8; // Green LED (non-magnetic object)
const int MAG_SENSOR_1 = 9; // Magnetic Sensor Pin
const int MAG_SENSOR_2 = 10;
const int LEFT_MOTOR_PIN = 2;
const int RIGHT_MOTOR_PIN = 3;
const unsigned long DEBOUNCE_DELAY = 80;
const unsigned long RIGHT_BOUNCE_DELAY = 90;
const unsigned long LEFT_BOUNCE_DELAY = 90;

// Variables
int currentPosition = DEFAULT_POSITION;
bool isActivated = false;
bool isMoving = false;
bool tof_active = true;
unsigned long tof_threshold;
bool object_detected = false;
int object_count = 0;
bool collected = false;
bool isMagnetic = false;
int count_split = -1;
int right_count = 0;
int left_count = 0;
int end = 0;
unsigned long rightDebounceTime = 0;
unsigned long leftDebounceTime = 0;
unsigned long splitDebounceTime = 0;

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(LEFT_MOTOR_PIN);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(RIGHT_MOTOR_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  
  Serial.println("Motor Shield found.");
  
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(RELEASE);
  Motor_Right->run(RELEASE);
  
  pinMode(EXTREME_LEFT_PIN, INPUT);
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  pinMode(EXTREME_RIGHT_PIN, INPUT);
  pinMode(INPUT_PIN, INPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(MAG_SENSOR_1, INPUT);
  pinMode(MAG_SENSOR_2, INPUT);
  
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  sensor.start();
  
  myservo.attach(SERVO_PIN);
  myservo.write(DEFAULT_POSITION);
}

void loop() {
  static bool motorsRunning = false;
  static int lastButtonState = LOW;
  
  int currentButtonState = digitalRead(INPUT_PIN);
  int distance = sensor.getDistance();
  
  Serial.print("Distance: ");
  Serial.println(distance);
  
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    delay(DEBOUNCE_DELAY);
    motorsRunning = !motorsRunning;
    
    if (motorsRunning) {
      run();
    } else {
      stopMotors();
    }
  }
  
  if (motorsRunning) {
    run();
    
    if (distance < THRESHOLD) {
      moveServo(ACTIVATED_POSITION);
      isActivated = true;
    }
    
    handleLEDs();
  }
  
  lastButtonState = currentButtonState;
}

void run() {
  int extreme_right = digitalRead(EXTREME_RIGHT_PIN);
  int right = digitalRead(RIGHT_PIN);
  int left = digitalRead(LEFT_PIN);
  int extreme_left = digitalRead(EXTREME_LEFT_PIN);
  
  path_follow(extreme_right, right, left, extreme_left);
  
  if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
    stopMotors();
    
    if (end == 0) {
      moveForward();
      delay(200);
      isMoving = true;
      
      while (!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)) {
        delay(10);
        extreme_right = digitalRead(EXTREME_RIGHT_PIN);
        right = digitalRead(RIGHT_PIN);
        left = digitalRead(LEFT_PIN);
        extreme_left = digitalRead(EXTREME_LEFT_PIN);
      }
      
      stopMotors();
    }
  }
  
  if (object_count == 1 && isMagnetic && collected) {
    while (collected) {
      extreme_right = digitalRead(EXTREME_RIGHT_PIN);
      right = digitalRead(RIGHT_PIN);
      left = digitalRead(LEFT_PIN);
      extreme_left = digitalRead(EXTREME_LEFT_PIN);
      
      path_follow(extreme_right, right, left, extreme_left);
      
      handleTurns(extreme_right, right, left, extreme_left);
    }
  }
  
  if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 0) {
    stopMotors();
    
    if (end == 1) {
      moveBackward();
      delay(200);
      
      while (!(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1)) {
        delay(10);
        extreme_right = digitalRead(EXTREME_RIGHT_PIN);
        right = digitalRead(RIGHT_PIN);
        left = digitalRead(LEFT_PIN);
        extreme_left = digitalRead(EXTREME_LEFT_PIN);
      }
      
      stopMotors();
    }
  }
}

void path_follow(int extreme_right, int right, int left, int extreme_left) {
  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0) {
    moveForward();
    if (!isMoving) {
      isMoving = true;
    }
  } else if (extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0) {
    shift_left(extreme_right, right, left, extreme_left);
  } else if (extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0) {
    shift_right(extreme_right, right, left, extreme_left);
  }
}

void handleLEDs() {
  digitalWrite(isActivated ? GREEN_LED_PIN : RED_LED_PIN, HIGH);
  digitalWrite(isActivated ? RED_LED_PIN : GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, HIGH);
}

void moveServo(int position) {
  myservo.write(position);
}

void shift_left(int extreme_right, int right, int left, int extreme_left) {
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(BACKWARD);
  Motor_Right->run(FORWARD);
}

void shift_right(int extreme_right, int right, int left, int extreme_left) {
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(BACKWARD);
}

void turn_left(int extreme_right, int right, int left, int extreme_left) {
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(BACKWARD);
  Motor_Right->run(FORWARD);
}

void turn_right(int extreme_right, int right, int left, int extreme_left) {
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  Motor_Left->run(FORWARD);
  Motor_Right->run(BACKWARD);
}

void stopMotors() {
  Motor_Right->run(RELEASE);
  Motor_Left->run(RELEASE);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void moveForward() {
  Motor_Right->setSpeed(255);
  Motor_Left->setSpeed(255);
  Motor_Right->run(FORWARD);
  Motor_Left->run(FORWARD);
}

void moveBackward() {
  Motor_Right->setSpeed(255);
  Motor_Left->setSpeed(225);
  Motor_Right->run(BACKWARD);
  Motor_Left->run(BACKWARD);
}

void handleTurns(int extreme_right, int right, int left, int extreme_left) {
  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 0) {
    if (millis() - rightDebounceTime >= RIGHT_BOUNCE_DELAY) {
      rightDebounceTime = millis();
      right_count++;
      Serial.print("Right Count ");
      Serial.println(right_count);
      if (right_count == 2 || right_count == 4) {
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
  } else {
    rightDebounceTime = millis();
  }

  if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 1) {
    if (millis() - leftDebounceTime >= LEFT_BOUNCE_DELAY) {
      leftDebounceTime = millis();
      left_count++;
      Serial.println(left_count);
      if (left_count == 1 || left_count == 3) {
        turn_left(extreme_right, right, left, extreme_left);
      }
    }
  } else {
    leftDebounceTime = millis();
  }

  if (extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1) {
    if (millis() - splitDebounceTime >= DEBOUNCE_DELAY) {
      splitDebounceTime = millis();
      count_split++;
      Serial.print("Split Count ");
      Serial.println(count_split);
      if (count_split == 1) {
        turn_left(extreme_right, right, left, extreme_left);
      } else if (count_split == 2 || count_split == 3) {
        turn_right(extreme_right, right, left, extreme_left);
      }
    }
  } else {
    splitDebounceTime = millis();
  }
}
