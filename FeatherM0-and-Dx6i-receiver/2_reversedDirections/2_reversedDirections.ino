
#include <Wire.h>

// speed (elevator)
const byte elevPin = 12;  // blue
unsigned long elevPrevTime;
unsigned int elevDuration = 1500;  // by default in case we don't have a reading yet
int elevOffset = 0;// so as not to affect the interrupt readings before we've
                     // gotten an actual offset value!
// steer (rudder)
const byte ruddPin = 11;  // white
unsigned long ruddPrevTime;
unsigned int ruddDuration = 1500;  // by default
int ruddOffset = 0;  

const byte buff = 20; // some leeway around center thumbstick position

// reset (gear switch)
const byte gearPin = 10;  // purple
unsigned long gearPrevTime;
unsigned int gearDuration = 1500;

unsigned int blinkDelay = 1000;  // blinks led 13, see setup
unsigned long blinkPrevTime;
byte blinkState = 0;  // starts off

// holds converted readings, sent via I2C
// speed, sign, steer, sign, reset
uint8_t data[5] = {0, 0, 0, 0, 0};

// calculating
unsigned long calcPrevTime;
const unsigned int calcInterval = 100; // to match with 101's request interval

// printing
unsigned long printPrevTime;
const int deley = 100;

// wait a bit after registering the interrupts before calculating
// the offset, so we know we have actual readings, not the default ones.
unsigned long interruptsRead;
const unsigned long offsetDelay = 50;

void setup(){
  Serial.begin(19200);
  //while(!Serial) ;
  
  /********** interrupts **********/
  pinMode(ruddPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ruddPin), ruddChange, CHANGE);
  
  pinMode(elevPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(elevPin), elevChange, CHANGE);

  interruptsRead = millis();

  pinMode(gearPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(gearPin), gearChange, CHANGE);
  pinMode(13, OUTPUT);
  // blink led slow when not resetting, fast when resetting!

  /********** I2C **********/
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event

  /********** calc stuff **********/
  // wait while we get some readings from the receiver...
  while (millis() - interruptsRead < offsetDelay) ; // just sit tight
  
  // get the offset for each potentiometer value.
  // Must register interrupts first, and have a reading!
  ruddOffset = 1500 - ruddDuration;
  elevOffset = 1500 - elevDuration;
  Serial.print("rudder offset: ");
  Serial.println(ruddOffset);
  Serial.print("elevator offset: ");
  Serial.println(elevOffset);

  calcPrevTime = millis();
  /********** printing **********/
  printPrevTime = millis();

  blinkPrevTime = millis();
}

void loop(){
  if (millis() - calcPrevTime > calcInterval) {
    // Map readings from reciever to numbers passable over I2C.
    // In loop instead of requestEvent so it's interruptable.
    
    
    // speed
    if (elevDuration >= 1500 + buff) {
      data[0] = map(elevDuration, 1500+buff, 1950, 0, 255);
      data[1] = 1; // negative b/c reversed
    }
    else if (elevDuration <= 1500 - buff) {
      // the shorter the high, the more negative (sign is separate) the number should be
      data[0] = map(elevDuration, 1050, 1500-buff, 255, 0);
      data[1] = 0; // not negative b/c reversed
    }
    else{ // it is within the buffer
      data[0] = 0;
      data[1] = 0;
    }
    
    
    // steering
    if (ruddDuration >= 1500 + buff) {
      data[2] = map(ruddDuration, 1500+buff, 1950, 0, 255);
      data[3] = 0; // negative b/c reversed
    }
    else if (ruddDuration <= 1500 - buff) {
      // the shorter the high, the more negative (sign is separate) the number should be
      data[2] = map(ruddDuration, 1050, 1500-buff, 255, 0);
      data[3] = 1; // not negative b/c reversed
    }
    else{ // it is within the buffer
      data[2] = 0;
      data[3] = 0;
    }
    
    /*Serial.print(elevDuration);
    Serial.print('\t');
    Serial.print(data[1]);
    Serial.print('\t');
    Serial.print(data[0]);
    Serial.print('\t');
    Serial.print(ruddDuration);
    Serial.print('\t');
    Serial.print(data[3]);
    Serial.print('\t');
    Serial.print(data[2]);u
    Serial.print('\t');
    Serial.print(gearDuration);
    Serial.print('\t');
    Serial.print(blinkDelay);
    Serial.print('\t');
    Serial.println(data[4]);*/

    calcPrevTime += calcInterval;
  } // end calcInterval

  if (millis() - blinkPrevTime > blinkDelay) {
    if (blinkState) {  // if it's currently on, switch it off
      digitalWrite(13, LOW);
      blinkState = 0;
    }
    else {  // otherwise it's currently off, so switch it on
      digitalWrite(13, HIGH);
      blinkState = 1;
    }

    blinkPrevTime += blinkDelay;
  }
  
  /**/
}


/********** I2C interrupt function **********/
// function that executes whenever data is requested by master (every 100 ms)
// this function is registered as an event, see setup()
void requestEvent() {
  //Serial.print("Sending over I2C... ");
  //Wire.beginTransmission(9);
  for(int i = 0; i < 5; i++){
    Wire.write(data[i]); // respond with message of 5 bytes
    //Serial.print(data[i]); space(data[i]);
  }
  //Wire.endTransmission();
  //Serial.println();
}

/********** Transmitter interrupt functions **********/  
void ruddChange(){
  if (digitalRead(ruddPin))  // if currently high, i.e. triggered rising transition
    ruddPrevTime = micros();  // get the current time it transitioned
  else
    ruddDuration = micros() - ruddPrevTime + ruddOffset;    
}

void elevChange(){
  if (digitalRead(elevPin))  // if currently high, i.e. triggered rising transition
    elevPrevTime = micros();  // get the current time it transitioned
  else
    elevDuration = micros() - elevPrevTime + elevOffset;
}

void gearChange(){
  if (digitalRead(gearPin))
    gearPrevTime = micros();
  else // low, so not resetting everything
    gearDuration = micros() - gearPrevTime;

  if (gearDuration < 1500){  // 0, gear, reseting
    data[4] = 1;
    blinkDelay = 100;
  }
  else{  // 1, flightmode, not reseting!
    data[4] = 0;
    blinkDelay = 800;
  }
}

