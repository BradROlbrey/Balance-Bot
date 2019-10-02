
const byte ruddPin = 12;
unsigned long ruddPrevTime;
unsigned int ruddDuration = 1234567890;

const byte throPin = 11;
unsigned long throPrevTime;
unsigned int throDuration = 1234567890;

unsigned long printPrevTime;
const int deley = 100;



void setup(){
  Serial.begin(19200);
  pinMode(ruddPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ruddPin), ruddChange, CHANGE);
  
  pinMode(throPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(throPin), throChange, CHANGE);

  printPrevTime = millis();
}

void loop(){
  while(millis() - printPrevTime > deley){
    Serial.print(ruddDuration);
    Serial.print('\t');
    Serial.println(throDuration);
    printPrevTime += deley;
  }
}



void ruddChange(){
  if(digitalRead(ruddPin))  // if currently high, i.e. triggered rising transition
    ruddPrevTime = micros();  // get the current time it transitioned
  else
    ruddDuration = micros() - ruddPrevTime;    
}
void throChange(){
  if(digitalRead(throPin))  // if currently high, i.e. triggered rising transition
    throPrevTime = micros();  // get the current time it transitioned
  else
    throDuration = micros() - throPrevTime;
}

