int leftlinesensorPin = 2;
int rightlinesensorPin = 3; 



void setup() {
  Serial.begin(9600); // Init the serial port
  pinMode(leftlinesensorPin, INPUT); // declare LED as output
  pinMode(rightlinesensorPin, INPUT); // declare Micro switch as input
  // put your setup code here, to run once:

}

void loop() {
   int valLeft = digitalRead(leftlinesensorPin); // read left input value
   Serial.print(valLeft);
   int valRight = digitalRead(rightlinesensorPin); // read right input value
   Serial.println(valRight);
   delay(100);

  // put your main code here, to run repeatedly:

}
