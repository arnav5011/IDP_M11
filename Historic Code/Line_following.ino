int leftlinesensorPin = 4;
int midlinesensorPin = 2;
int rightlinesensorPin = 3; // Connect sensor to input pin 3
void setup() {
 Serial.begin(9600); // Init the serial port
 pinMode(leftlinesensorPin, INPUT);
 pinMode(midlinesensorPin, INPUT); // declare LED as output
 pinMode(rightlinesensorPin, INPUT); // declare Micro switch as input
}
void loop(){
 int valLeft = digitalRead(leftlinesensorPin);
 Serial.print(valLeft);
 int valMid = digitalRead(midlinesensorPin); // read left input value
 Serial.print(valMid);
 int valRight = digitalRead(rightlinesensorPin); // read right input value
 Serial.println(valRight);
 delay(100);
}