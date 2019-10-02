
// Feather M0 on the Bot
// uses RFM95 to receive data from the Feather on the Transmitter
// uses I2C to send data to the Arduino 101 on the Bot

#include <Wire.h> // I2C

#include <SPI.h> // LoRa
#include <RH_RF95.h>
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver

// receives this over air and passes them on via I2C
// speed, sign, steer, sign, reset
uint8_t data[5] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);           // start serial for output
  while (!Serial) ;
  
  /********** I2C **********/
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  
  /********** LoRa **********/
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); // not resetting
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  rf95.setTxPower(23, false);
  Serial.print("Set Transmit power to: "); Serial.println(23);
}

void loop() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t len = sizeof(data);
    
    if (rf95.recv(data, &len)) { // store received stuff
      digitalWrite(13, HIGH);
      Serial.print("Got: ");
      for(int i = 0; i < 5; i++) {
        Serial.print(data[i]);
        space(data[i]);
      }
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(13, LOW);
    }
    else {
      Serial.println("Receive failed");
    }
    Serial.println();
  } // end if available
  
}

// function that executes whenever data is requested by master
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

void space(int num) {
  String string = String(num);
  int spaces = 8 - string.length();
  for(int i = 0; i < spaces; i++) {
    Serial.print(' ');
  }
}


