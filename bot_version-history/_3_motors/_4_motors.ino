
// Bot

#include <CurieBLE.h>

/*********** Bluetooth stuff ***********/
// BLE peripheral object
BLEPeripheral blePeripheral;
char device_name[] = "Balance Bot";

BLECentral central = blePeripheral.central();

// characteristic properties
// Bot will receive these characteristics from the Transmitter,
// so we must make them writable by the Transmitter.
int props = BLEWrite | 0;

// Services and Characteristics
BLEService Bot("3E099910-294F-11E5-93BE-AFD1FE6D1DFD");
BLEIntCharacteristic Gyro_Prop("3E099911-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Gyro_Diff("3E099912-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Prop("3E099913-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Diff("3E099914-294F-11E5-93BE-AFD1FE6D1DFD", props);

// Bot stuff
int gyro_prop = 0; // proportional of gyro
int gyro_diff = 0; // differential of gyro
int disp_prop = 0; // proportional of displacement
int disp_diff = 0; // differential of displacement


/*********** Motor stuff ***********/
// motor 1
const byte in1 = 2; // blue;    out1 = orange
const byte in2 = 4; // green;   out2 = yellow
const byte ena1 = 3; // yellow

// motor 2
// to make things easier, I've simply flipped around the 3 pins
// that go into the arduino, reversing its direction without having
// to rewrite any code.
const byte in3 = 5; // blue; actually green    out3 = orange
const byte in4 = 7; // green; actually blue   out4 = yellow
const byte ena2 = 6; // yellow; still yellow



void setup() {
  Serial.begin(9600);
  while(!Serial) ; // wait until serial monitor is open

/*********** Motor stuff ***********/
  // motor 1
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena1, OUTPUT);
  // motor 2
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena2, OUTPUT);
  // start braked
  brakes();


/*********** Bluetooth stuff ***********/
  // Bluetooth
  // set advertising packet content
  blePeripheral.setLocalName(device_name);
  blePeripheral.setAdvertisedServiceUuid(Bot.uuid());

  // add services and characteristics
  blePeripheral.addAttribute(Bot);
  blePeripheral.addAttribute(Gyro_Prop);
  blePeripheral.addAttribute(Gyro_Diff);
  blePeripheral.addAttribute(Disp_Prop);
  blePeripheral.addAttribute(Disp_Diff);
  
  // begin advertising
  blePeripheral.begin();
  Serial.println("advertising Bot profile");

}

unsigned long time_prev = millis();

void loop() {
  
/*********** Bluetooth stuff ***********/
  // if we do not have a central, or
  // if we have a central but we are disconnected,
  if (!central || !central.connected()) {
    // try to get a central
    Serial.print("Looking\t\t");
    central = blePeripheral.central();
  } // else
  // if we have a central, and we are connected
  else if (central && central.connected()) {
    Serial.print("connected\t");
    
    // do bluetooth stuff
    if (Gyro_Prop.written()) 
      gyro_prop = Gyro_Prop.value();
    if (Gyro_Diff.written())
      gyro_diff = Gyro_Diff.value();
    if (Disp_Prop.written())
      disp_prop = Disp_Prop.value();
    if (Disp_Diff.written())
      disp_diff = Disp_Diff.value();
    
  } // else, we are not supposed to be here
  //else Serial.print("Bluetooth problem");


/*********** Motor PID stuff ***********/
  int tilt = 0, target = 0, error = 0, result = 0;
  // if the bot is not completely tipped over
  if (tilt > -60 && tilt < 60) {
    error = tilt - target; // error in tilt from 90 degrees
    //result = error * prop + tilt_change * diff;
    result = error * gyro_prop;
    // take the absolute value only after all the math!
    // if reverse direction only by error, abs val of result means the motor just keeps spinning
    if (result > 0) { // target greater than current pos. We are behind target, 
      // set direction to forwards
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      
      // make motor move in proportion to distance from target
      analogWrite(ena1, abs(result));
      analogWrite(ena2, abs(result));
    } // else
    else if (result < 0) { // target less than current pos. We are in front of target,
      // set direction to backwards
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      
      // make motor move in proportion to distance from target
      analogWrite(ena1, abs(result));
      analogWrite(ena2, abs(result));
    } // end if
    else{ // if we are within the tolerance, at an appropriate postion...
      // full brakes
      brakes();
    } // end else
  } // end if not tipped over
  // else if it is tipped over
  else
    // hit the brakes!
    brakes();

}



void brakes(){
  digitalWrite(ena1, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(ena2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void spaces(int num){
  String string = String(num);
  short numSpaces = 8 - string.length();
  String space = "";
  for(int i = 0; i < numSpaces; i++){
    space += " ";
  }
  Serial.print(space);
}


