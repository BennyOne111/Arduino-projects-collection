// DC Motor with Encoder
// Author: Yiyang Jiang
volatile int counter = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  // Have interrupt routines monitor two digital input pins for CHANGEs in the ChA and ChB encoder output signals
  attachInterrupt(0, changeA, CHANGE);
  attachInterrupt(1, changeB, CHANGE);
}

void changeA() {
  if (digitalRead(2) != digitalRead(3)){
    // if ChA and ChB are different (one HIGH and one LOW),
    // add one to a counter variable;
    counter++;
  }
  else{
    // if ChA and ChB are same,
    // substract one to a counter variable;
    counter--;
  }
}

void changeB() {
  if (digitalRead(2) == digitalRead(3)){
    // if ChA and ChB are same,
    // add one to a counter variable;
    counter++;
  }
  else{
    // if ChA and ChB are different,
    // substract one to a counter variable;
    counter--;
  }
}

void loop() {
  Serial.println(counter);
}
