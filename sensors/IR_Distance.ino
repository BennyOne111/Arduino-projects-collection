// IR Distance
// Author: Yiyang Jiang

// Set the input pin
const int PIN = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read the analog data from A0
  float voltage = 5/1024*analogRead(PIN);
  // Calculate the distance
  float L = 20/(voltage-0.3);
  Serial.println("test");
  Serial.println(voltage);
  Serial.println(L);
  delay(5000);
}
