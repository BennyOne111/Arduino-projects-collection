// Full-Step Motor Control 
// Author: Yiyang Jiang

// The number of phase coils for the motor
const int NUMPHASES = 4;
// The pins on Arduino connected to the phases
const int PHASEPINS[NUMPHASES] = {9, 10, 11, 12};
// The number of increments per step, full step
const int INCREMENTSPERSTEP = 1;  
// The totel increments per revolution
const int NUMINCREMENTS = NUMPHASES*INCREMENTSPERSTEP;
// The number of degrees per step for the motor
const int DEGREEPERSTEP;
// The delay time between increments
const int DELAYTIME = 100;
// A two-dimensional array for the state of each pole for each increment in a
// complete cycle of the four phase coils.
const bool POLESONOFF[NUMINCREMENTS][NUMPHASES] = {
  {HIGH, HIGH, LOW, LOW}, // Step 1
  {LOW, HIGH, HIGH, LOW}, // Step 2 
  {LOW, LOW, HIGH, HIGH}, // Step 3 
  {HIGH, LOW, LOW, HIGH} // Step 4
  //{HIGH, LOW, LOW, LOW}, // Step 1
  //{HIGH, HIGH, LOW, LOW}, // Step 2
  //{LOW, HIGH, LOW, LOW}, // Step 3
  //{LOW, HIGH, HIGH, LOW}, // Step 4
  //{LOW, LOW, HIGH, LOW}, // Step 5
  //{LOW, LOW, HIGH, HIGH}, // Step 6 
  //{LOW, LOW, LOW, HIGH} // Step 7
  //{HIGH, LOW, LOW, HIGH} // Step 8
};
// The pin of the button
const int SWITCHPIN = 6;
// The current increment
int CURRENTINCREMENT = 0;

const int IRPin = A0;

void setup() {
  // put your setup code here, to run once:
  for (int phase=0; phase < NUMPHASES; phase++){
    pinMode(PHASEPINS[phase], OUTPUT);
    digitalWrite(PHASEPINS[phase], LOW);
  }
  pinMode(SWITCHPIN, INPUT_PULLUP);
}

void loop() {
// To keeo the current increment stay in the four possible increments
  CURRENTINCREMENT = (CURRENTINCREMENT+1) % NUMINCREMENTS;
  // Turn each the four phase pins on or off, according to the pattern for full steps
  for (int phase=0; phase<NUMPHASES; phase++){
    digitalWrite(PHASEPINS[phase], POLESONOFF[CURRENTINCREMENT][phase]);
  }

  // put your main code here, to run repeatedly:
  // Read the analog data from A0
  float voltage = 5/1024*analogRead(IRPin);
  // Calculate the distance
  float L = 20/(voltage-0.3);
  Serial.print("The distance is: ");
  Serial.println(L);
  // Delay for the appropriate time
  delay(DELAYTIME);
}
