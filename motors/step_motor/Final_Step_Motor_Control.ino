// Full-Step Motor Control with button, one full circle per push of button
// Author: Yiyang Jiang

// The number of phase coils for the motor
const int NUMPHASES = 4;
// The pins on Arduino connected to the phases
const int PHASEPINS[NUMPHASES] = {2, 3, 4, 5};
// The number of increments per step
const int INCREMENTSPERSTEP = 1;  
// The totel increments per revolution
const int NUMINCREMENTS = NUMPHASES*INCREMENTSPERSTEP;
// The number of degrees per step for the motor
const int DEGREEPERSTEP;
// 0.5s after doing a single step or switching the direction
const int DELAYTIME1 = 50;
// Delay for 50ms between steps when moving 360 degrees.
const int DELAYTIME2 = 500;
// const int DELAYTIME3 = 200;
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
// The current increment
int CURRENTINCREMENT = 0;
// The pin of one-step button
const int ONESTEP_SWITCHPIN = 6;
// The pin of one-circle button
const int ONECIRCLE_SWITCHPIN = 7;
// The pin of "reverse" button
const int REVERSE_SWITCHPIN = 8;
// The direction
int dir = 1;

void setup() {
  for (int phase=0; phase < NUMPHASES; phase++){
    pinMode(PHASEPINS[phase], OUTPUT);
    digitalWrite(PHASEPINS[phase], LOW);
  }
  // Initialize the pins of buttons
  pinMode(ONESTEP_SWITCHPIN, INPUT_PULLUP);
  pinMode(ONECIRCLE_SWITCHPIN, INPUT_PULLUP);
  pinMode(REVERSE_SWITCHPIN, INPUT_PULLUP);
}

void loop() {
  // The one-step button is pressed
  if (digitalRead(ONESTEP_SWITCHPIN) == LOW){
    // Identify the direction
    if (dir == 1){
      // Move forward
      CURRENTINCREMENT = (CURRENTINCREMENT+1) % NUMINCREMENTS;
      for (int phase=0; phase<NUMPHASES; phase++){
        digitalWrite(PHASEPINS[phase], POLESONOFF[CURRENTINCREMENT][phase]);
      }
    }  
    if (dir == -1){
      // Check if the URRENTINCREMENT goes negative, and if so, fix it.
      if (CURRENTINCREMENT == 0){
        CURRENTINCREMENT = 4;
      }
      // Move backward
      CURRENTINCREMENT = (CURRENTINCREMENT-1) % NUMINCREMENTS;
      for (int phase=0; phase<NUMPHASES; phase++){
        digitalWrite(PHASEPINS[phase], POLESONOFF[CURRENTINCREMENT][phase]);
      }
    }
    // 0.5s after doing a single step
    delay(DELAYTIME2);
  }

  // The one-circle button is pressed
  if (digitalRead(ONECIRCLE_SWITCHPIN) == LOW){
    if (dir == 1){
      // Move forward 12 steps(360/7.5/4), each step includes four increment, each increment represents 7.5 degree
      for (int step = 0; step < 12; step++){
        for (int increment = 0; increment < NUMINCREMENTS; increment++){
          // Set each of the four phase pins to OUTPUT at a LOW level
          for (int phase=0; phase<NUMPHASES; phase++){
            digitalWrite(PHASEPINS[phase], POLESONOFF[increment][phase]);
          }
          // A 50ms delay between increments
          delay(DELAYTIME1);
        }
      }
    }
    if (dir == -1){
      // Move backward 12 steps(360/7.5/4), each step includes four increment, each increment represents 7.5 degree
      for (int step = 0; step < 12; step++){
        for (int increment = NUMINCREMENTS - 1; increment >= 0; increment--){
          for (int phase=0; phase<NUMPHASES; phase++){
            digitalWrite(PHASEPINS[phase], POLESONOFF[increment][phase]);
          }
          delay(DELAYTIME1);
        }
      }
    }
  }

  // The "reverse" button is pressed
  if (digitalRead(REVERSE_SWITCHPIN) == LOW){
    // Change the direction
    dir = (-1)*dir;
    // 0.5s after switching the direction
    delay(DELAYTIME2);
  }
}
