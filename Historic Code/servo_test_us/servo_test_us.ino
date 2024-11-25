#include <Servo.h>

Servo myservo;  // Create servo object to control a servo
// Twelve servo objects can be created on most boards
int pos = 0;    // Variable to store the servo position

#define MAX_RANG (520)      // The max measurement value of the module is 520cm
#define ADC_SOLUTION (1023.0) // ADC accuracy of Arduino UNO is 10-bit
int sensityPin = A1;        // Select the input pin for the ultrasonic sensor

void setup() {
  myservo.attach(12);        // Attach the servo on pin 10 to the servo object
  Serial.begin(9600);        // Initialize serial communication
}

float dist_t, sensity_t;

void loop() {
  // Read the sensor value
  sensity_t = analogRead(sensityPin);  
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;  // Calculate the distance in cm
  
  Serial.print(dist_t, 0);  // Print the distance
  Serial.println(" cm");
  
  // If a box is detected (distance less than a threshold, e.g., 20 cm)
  if (dist_t < 10) {  
    // Move the servo from 0 to 180 degrees
    for (pos <= 180; pos += 1) { 
      myservo.write(pos);    // Tell servo to go to position 'pos'
      delay(15);             // Wait 15 ms for the servo to reach the position
    }
  }

  else {
    // Move the servo back from 180 to 0 degrees
    for (pos >= 0; pos -= 1) { 
      myservo.write(pos);    // Tell servo to go to position 'pos'
      delay(15);             // Wait 15 ms for the servo to reach the position
    }
  }
}
