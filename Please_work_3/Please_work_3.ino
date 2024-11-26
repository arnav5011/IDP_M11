#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define motor ports
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(2);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(3);

// Define sensor pins
const int extreme_right_pin = 2;
const int right_pin = 3;
const int left_pin = 4;
const int extreme_left_pin = 5;
const int input_pin = 11;
const int magnetSensorPin = 9; // Assuming magnet sensor is on pin 9

// Define LED pins
const int blueLED = 6;
const int redLED = 7;
const int greenLED = 8;

// Timing variables for flashing
unsigned long previousMillis = 0;
const long interval = 100; // 5Hz blinking (100ms on/off)

// Other variables
bool isMoving = false;
int count_split = 0;
int right_count = 0;
int left_count = 0;
int end = 0;

// Debounce variables
unsigned long debounceDelay = 80;
unsigned long rightbounceDelay = 90;
unsigned long leftbounceDelay = 90;
unsigned long rightDebounceTime = 0;
unsigned long leftDebounceTime = 0;
unsigned long splitDebounceTime = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("Adafruit Motorshield v2 - Integrated Motor and LED Control");

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

    // Set up pin modes
    pinMode(extreme_left_pin, INPUT);
    pinMode(left_pin, INPUT);
    pinMode(right_pin, INPUT);
    pinMode(extreme_right_pin, INPUT);
    pinMode(input_pin, INPUT);
    pinMode(magnetSensorPin, INPUT);

    // Set up LED pins
    pinMode(blueLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    pinMode(greenLED, OUTPUT);
}

void loop() {
    static bool motorsRunning = true; // Track the state of the motors
    static unsigned long lastDebounceTime = 0; // For debouncing
    const unsigned long debounceDelay = 50; // Debounce delay

    int pin_val = digitalRead(input_pin);

    // Check if button state has changed
    if (pin_val == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = millis(); // Update the last debounce time

        // Toggle motor state
        motorsRunning = !motorsRunning;

        if (motorsRunning) {
            move(); // Start moving
        } else {
            Motor_Right->run(RELEASE); // Stop motors
            Motor_Left->run(RELEASE);
            digitalWrite(blueLED, LOW); // Turn off blue LED when stopped
        }
    }

    // If motors are running, continue executing move logic
    if (motorsRunning) {
        move();
        flashBlueLED(); // Flash blue LED when moving
    }

    // Check magnet sensor and control red/green LEDs
    if (digitalRead(magnetSensorPin) == HIGH) {
        digitalWrite(redLED, HIGH);
        digitalWrite(greenLED, LOW);
    } else {
        digitalWrite(redLED, LOW);
        digitalWrite(greenLED, HIGH);
    }
}

void move() {
    int extreme_right = digitalRead(extreme_right_pin); 
    int right = digitalRead(right_pin);
    int left = digitalRead(left_pin);
    int extreme_left = digitalRead(extreme_left_pin);

    if (extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0) { 
        Motor_Left->setSpeed(255);
        Motor_Right->setSpeed(255);
        Motor_Left->run(FORWARD);
        Motor_Right->run(FORWARD);
        isMoving = true;
        
    } else if (extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0) {
        shift_left(extreme_right, right, left, extreme_left); 
      
      } else if (extreme_right == 0 && right == 0 && left == 1 && extreme_left == 1) {
          shift_left(extreme_right, right, left, extreme_left); 
      
      } else if (extreme_right == 0 && right == 0 && left == 0 && extreme_left == 1) {
          shift_left(extreme_right, right, left, extreme_left); 
      
      } else if (extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0) {
          shift_right(extreme_right, right, left, extreme_left); 
      
      } else if (extreme_right == 1 && right == 1 && left == 0 && extreme_left == 0) {
          shift_right(extreme_right, right, left, extreme_left); 
      
      } else if (extreme_right == 1 && right == 0 && left == 0 && extreme_left == 0) {
          shift_right(extreme_right, right, left, extreme_left); 
      
      }

      if (extreme_right==1 && right==1 && left==1 && extreme_left==1){
          splitDebounceTime=millis();
          count_split++;
          Serial.print("Split Count ");
          Serial.println(count_split);

          if(count_split==1 || count_split==3 || count_split==4){
              turn_left(extreme_right,right,left,extreme_left);
              
          }else if(count_split==2 || count_split==5){
              turn_right(extreme_right,right,left,extreme_left);

          }
          
      }else{
          splitDebounceTime=millis();
          
      }

      if(extreme_right==1&&right==1&&left==1&&extreme_left==0){
          rightDebounceTime=millis();
          right_count++;
          Serial.print("Right Count ");
          Serial.println(right_count);

          if(right_count==2||right_count==3||right_count==4||right_count==5){
              turn_right(extreme_right,right,left,extreme_left);

          }
          
      }else{
          rightDebounceTime=millis();
          
      }

      if(extreme_right==0&&right==1&&left==1&&extreme_left==1){
          leftDebounceTime=millis();
          left_count++;
          Serial.println(left_count);

          if(left_count==1){
              turn_left(extreme_right,right,left,extreme_left);

          }
          
      }else{
          leftDebounceTime=millis();
          
      }
    
}

void flashBlueLED() {
    unsigned long currentMillis = millis();
    
     if(currentMillis-previousMillis>=interval){
         previousMillis=currentMillis;

         digitalWrite(blueLED,!digitalRead(blueLED));
         
     }
    
}


void turn_right(int extreme_right,int right,int left,int extreme_left){
     Motor_Left->setSpeed(255);
     Motor_Right->setSpeed(255);

     Motor_Left->run(FORWARD);
     Motor_Right->run(FORWARD);

     delay(500);

     Motor_Right->run(BACKWARD);

     delay(500);

     isMoving=true;

     while(!(extreme_right==0&&right==1&&left==1&&extreme_left==0)){
         delay(10);

         extreme_right=digitalRead(extreme_right_pin);
         right=digitalRead(right_pin);
         left=digitalRead(left_pin);
         extreme_left=digitalRead(extreme_left_pin);

         
     }

     Motor_Right->run(FORWARD);


     
}

void turn_left(int extreme_right,int right,int left,int extreme_left){
    
     Motor_Left->setSpeed(255);
     Motor_Right->setSpeed(255);

     Motor_Left->run(FORWARD);
     Motor_Right->run(FORWARD);

     delay(500);

     Motor_Left->run(BACKWARD);

     delay(500);

     isMoving=true;

     while(!(extreme_right==0&&right==1&&left==1&&extreme_left==0)){
         delay(10);

         extreme_right=digitalRead(extreme_right_pin);
         right=digitalRead(right_pin);
         left=digitalRead(left_pin);
         extreme_left=digitalRead(extreme_left_pin);


         
     }

     Motor_Left->run(FORWARD);


     
}

void shift_right(int extreme_right,int right,int left,int extreme_left){

   Motor_Left->setSpeed(255);
   Motor_Right->setSpeed(255);


   Motor_Left->run(FORWARD);
   Motor_Right->run(FORWARD);


   Motor_Right->setSpeed(0);


   isMoving=true;


   
}

void shift_left(int extreme_right,int right,int left,int extreme_left){

   Motor_Left->setSpeed(255);
   Motor_Right->setSpeed(255);


   Motor_Left->run(FORWARD);
   Motor_Right->run(FORWARD);


   Motor_Left->setSpeed(0);


   isMoving=true;


   
}
