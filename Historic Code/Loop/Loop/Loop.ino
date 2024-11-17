#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *Motor_Left = AFMS.getMotor(4);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(1);

int extreme_right_pin;
int right_pin;
int left_pin;
int extreme_left_pin;

int blue_led_pin;
int green_led_pin;



void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  Motor_Left->setSpeed(255);
  Motor_Right->setSpeed(255);
  pinMode(2,INPUT);  //Extreme Right
  pinMode(3, INPUT); //Right
  pinMode(4,INPUT);  //Left 
  pinMode(5,INPUT);  //Extreme Left
  pinMode(6, OUTPUT); //Blue LED Module - For Motion
  pinMode(7, OUTPUT); //Green LED Module - For Magnetic
  pinMode(8, INPUT); //Magnetic Sensor
  pinMode(9, INPUT); //Ultrasonic Sensor
  pinMode(10, INPUT); //Time of Flight Sensor

}

void loop() {
  int extreme_right = digitalRead(5);
  int right = digitalRead(4);
  int left = digitalRead(3);
  int extreme_left = digitalRead(2);
  bool isMoving = false;
  bool isMagnetic = false;
  bool obstacle = false;
  Serial.print(extreme_right);
  Serial.print(right);
  Serial.print(left);
  Serial.println(extreme_left);
  if(extreme_right == 0 && right == 1 && left == 1 && extreme_left == 0 ){
    Serial.println("Run Forward");
    Motor_Right->run(FORWARD);
    Motor_Left->run(FORWARD);
    isMoving = true;
  }
  else if(extreme_right == 0  && right == 1 && left == 1 && extreme_left == 1){ //Turn Left
    delay(100);
    Motor_Left->run(RELEASE);
    delay(1000);
    isMoving = true;
  }
  else if(extreme_right == 1  && right == 1 && left == 1 && extreme_left == 0){ //Turn Right
    delay(100);
    Motor_Right->run(RELEASE);
    delay(1000);
    isMoving = true;
  }
  else if(extreme_right == 0 && right == 0 && left == 1 && extreme_left == 0) { //Turn slightly left
    Motor_Left->setSpeed(200);
    delay(250);
    isMoving = true;
    Motor_Left->setSpeed(250);
  }
  else if(extreme_right == 0 && right == 1 && left == 0 && extreme_left == 0) { //Turn slightly right
    Motor_Right->setSpeed(200);
    delay(50);
    isMoving = true;
    Motor_Right->setSpeed(250);
  }
  else if(extreme_right == 1 && right == 1 && left == 1 && extreme_left == 1){ //Turn Right at 1111 for time being
    delay(250);
    Motor_Right->run(RELEASE);
    delay(500);
    isMoving = true;
  }
  else{ //Stop Running
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
    isMoving = false;
  }

  if (isMoving){
    digitalWrite(6, HIGH);
    delay(100);
    digitalWrite(6, LOW);
  }

}
