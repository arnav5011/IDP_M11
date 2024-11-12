#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer
//han effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit
int sensityPin = A1; // select the input pin



void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);

}
float dist_t, sensity_t;

void loop() {
  // read the value from the sensor:
  sensity_t = analogRead(sensityPin);
 // turn the ledPin on
  dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;//
  Serial.print(dist_t,0);
  Serial.println("cm");
  delay(500);

  // put your main code here, to run repeatedly:

}
