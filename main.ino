/*
    ALL CURRENT PINS AND SIMILAR FEATURES ARE ASSIGNED ARBITRARILY IF AT ALL
*/

/*                  ----    Preamble    ----                */
/*  ----    TIMING FOR INTERRUPTS    ----    */
#include <TimerOne.h>

#define ledInterval 250000

/*  ----    ADAFRUIT MOTORSHIELD    ----    */
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Selecting ports for motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor();
Adafruit_DCMotor *rightMotor = AFMS.getMotor();

int movementSpeed = 150;
bool isMoving = false;

/*  ----    Sensors    ----    */
#define leftmostLineSensor 
#define leftMiddleLineSensor 
#define rightMiddleLineSensor 
#define rightmostLineSensor 

/*  ----    LEDs    ----    */
//Currently defined with arbitary pins
#define redLED 
#define greenLED 
#define blueLED 

/*                  ----    CODE    ----                */
/*  ----    Core Code    ----    */
void setup() {
    Serial.begin(9600);

    //setup leds for signalling
    initialise_leds();

    //setup motor
    initialise_motor();

    //setup line censors
    initialise_line_sensor();

    //setup up timer interrupt
    initialise_timer_interrupt();
}

void loop() {

}

/*  ----    Initialisation Functions    ----    */
void initialise_leds() {
    pinMode(redLED, OUTPUT);  // Set redLED pin as output
    digitalWrite(redLED, LOW); // Turn off the LED initially
    pinMode(greenLED, OUTPUT);  // Set greenLED pin as output
    digitalWrite(greenLED, LOW); // Turn off the LED initially
    pinMode(blueLED, OUTPUT);   // Set blueLED pin as output
    digitalWrite(blueLED, LOW);  // Turn off the LED initially
    Serial.println("All LEDs initialised as output and set at low.");
}

void initialise_motor() {
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Motor Shield found.");

    // Set the speed at start to 0
    //Set up left motor
    leftMotor->setSpeed(0);
    leftMotor->run(RELEASE);
    //Set up right motor
    rightMotor->setSpeed(0);
    rightMotor->run(RELEASE);

    Serial.println("Both motors are set up with speed 0.");
}

void initialise_line_sensor() {
    pinMode(leftmostLineSensor, INPUT);
    pinMode(leftMiddleLineSensor, INPUT);
    pinMode(rightMiddleLineSensor, INPUT);
    pinMode(rightmostLineSensor, INPUT);
    Serial.println("Line Sensor are setup as inputs.");
}

void initialise_timer_interrupt(){
    Timer1.initialize(ledInterval); // Set interval to 250 ms (250,000 us)
    Timer1.attachInterrupt(movement_toggle);      // Attach the toggle function as an interrupt
}

/*  ----    Utils    ----    */
void movement_toggle() {
    if(isMoving) {
        toggle_LED(blueLED);
    }
}

void toggle_LED(int pin) {
    int state = digitalRead(pin);  // Read the current state
    digitalWrite(pin, !state);     // Toggle the state
}

void move_forward_until_split() {
    Serial.println("Moving forward.");

    leftMotor->setSpeed(movementSpeed);
    rightMotor->setSpeed(movementSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    isMoving = true;

    while(true) {
        if (detect_split()) {
            Serial.println("Split detected - stopping.");
            break;  // Exit the loop when a split is detected
        }

        delay(500);
    }

    isMoving = false;
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    Serial.println("Vehicle stopped.");
}

bool detect_split() {
    int leftOuterReading = digitalRead(leftmostLineSensor);
    int rightOuterReading = digitalRead(rightmostLineSensor);

    if (leftOuterReading == HIGH && rightOuterReading == HIGH) {
        return true;  //Split detected
    }

    return false; //No split detected
}

