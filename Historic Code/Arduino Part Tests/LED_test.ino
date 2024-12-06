int led = 2;


void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT); //Set Pin3 as output

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led, HIGH); //Turn off led
  delay(2000);
  digitalWrite(led, LOW); //Turn on led
  delay(2000);


}
