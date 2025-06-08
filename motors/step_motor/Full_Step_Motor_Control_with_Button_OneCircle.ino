// Full-Step Motor Control with button, one full circle per push of button
// Author: Yiyang Jiang

// The number of phase coils for the motor
const int NUMPHASES = 4;
// The pins on Arduino connected to the phases
const int PHASEPINS[NUMPHASES] = {2, 3, 4, 5};
// The number of increments per step, full step
const int INCREMENTSPERSTEP = 1;  
// The totel increments per revolution
const int NUMINCREMENTS = NUMPHASES*INCREMENTSPERSTEP;
// The number of degrees per step for the motor
const int DEGREEPERSTEP;
// The delay time between increments
const int DELAYTIME = 50;
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

void setup() {
  // put your setup code here, to run once:
  for (int phase=0; phase < NUMPHASES; phase++){
    pinMode(PHASEPINS[phase], OUTPUT);
    digitalWrite(PHASEPINS[phase], LOW);
  }
  pinMode(SWITCHPIN, INPUT_PULLUP);
}

void loop() {
  // Identify if the button is pressed
  while (digitalRead(SWITCHPIN) == HIGH){
  }
  // Move forward 12 steps(360/7.5/4), each step includes four increment, each increment represents 7.5 degree
  for (int step = 0; step < 12; step++){
    for (int increment = 0; increment < NUMINCREMENTS; increment++){
      // Set each of the four phase pins to OUTPUT at a LOW level
      for (int phase=0; phase<NUMPHASES; phase++){
        digitalWrite(PHASEPINS[phase], POLESONOFF[increment][phase]);
      }
      // A 50ms delay between increments
      delay(DELAYTIME);
    }
  }
  //CURRENTINCREMENT = (CURRENTINCREMENT+1) % NUMINCREMENTS;
  delay(DELAYTIME);
}
