
// Bot

#include <CurieBLE.h>
#include <CurieIMU.h>

/*********** Bluetooth stuff ***********/
// BLE peripheral object
BLEPeripheral blePeripheral;
const char device_name[] = "Balance Bot";

BLECentral central = blePeripheral.central();

// characteristic properties
// Bot will receive these characteristics from the Transmitter,
// so we must make them writable by the Transmitter.
const int props = BLEWrite | 0;

// Services and Characteristics
BLEService Bot("3E099910-294F-11E5-93BE-AFD1FE6D1DFD");
BLEIntCharacteristic Gyro_Prop("3E099911-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Gyro_Diff("3E099912-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Prop("3E099913-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Diff("3E099914-294F-11E5-93BE-AFD1FE6D1DFD", props);

// Bot stuff
int gyro_prop = 1; // proportional of gyro
int gyro_diff = 0; // differential of gyro
int disp_prop = 0; // proportional of displacement
int disp_diff = 0; // differential of displacement


/*********** Motor stuff ***********/
// motor 1
const byte in1 = 4; // green;   out1 = orange
const byte in2 = 2; // blue;    out2 = yellow
const byte ena1 = 3; // yellow
/*
// motor 2
// to make things easier, I've simply flipped around the 3 pins
// that go into the arduino, reversing its direction without having
// to rewrite any code.
const byte in3 = 5; // blue; actually green    out3 = orange
const byte in4 = 7; // green; actually blue    out4 = yellow
const byte ena2 = 6; // yellow; still yellow
*/

/*********** Gyro stuff ***********/
const int updateRate = 100; // update frequency: x times per second
const int updateTime = 1000/updateRate; // ms per update
unsigned long gyro_timePrev = 0;
const int range = 125; // deg/sec
const byte axis = 0;
float gyro_read = 0;
float gyro_calib = 0;
float gyro_disp = 0;
float gyro_result = 0;


/*********** Print stuff ***********/
unsigned long print_timePrev = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial) ; // wait until serial monitor is open

/*********** Gyro stuff ***********/
  CurieIMU.begin();
  CurieIMU.setGyroRate(updateRate);
  // set gyro to lowest range for maximum accuracy!
  CurieIMU.setGyroRange(range);
  
  // calibrate gyro
  Serial.println("Calibrating");
  float gyro_temp = 0;
  int polls = 1, targetPolls = 500;
  // reset gyro_timePrev
  gyro_timePrev = millis();
  while (polls <= targetPolls) { // number of polls
    if (millis() - gyro_timePrev >= updateTime) {
      gyro_temp += CurieIMU.readGyroScaled(axis);
      gyro_timePrev += updateTime;
      polls++;
    }
  }
  gyro_calib = gyro_temp / targetPolls;

/*********** Motor stuff ***********/
  // motor 1
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena1, OUTPUT);
  /*// motor 2
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena2, OUTPUT);*/
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

  print_timePrev = millis();
}


void loop() {
/*********** Bluetooth stuff ***********/
  // if we do not have a central, or
  // if we have a central but we are disconnected,
  if (!central || !central.connected()) {
    // try to get a central
    //Serial.print("Looking\t\t");
    central = blePeripheral.central();
  } // else
  // if we have a central, and we are connected
  else if (central && central.connected()) {
    //Serial.print("connected\t");
      
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

/*********** Gyro stuff ***********/
  if (millis() - gyro_timePrev > updateTime) {
    gyro_timePrev += updateTime;
    gyro_read = CurieIMU.readGyroScaled(axis) - gyro_calib;
    gyro_disp += gyro_read / updateRate;


/*********** Motor PID stuff ***********/
    // if the bot is not completely tipped over
    if (gyro_disp > -60 && gyro_disp < 60) {
      //result = error * prop + tilt_change * diff;
      gyro_result = gyro_disp * gyro_prop;
      
      if (gyro_result > 0) { // leaning forward
        // set direction to forwards
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        /*digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);*/
        
        // make motor move in proportion to distance from target
        analogWrite(ena1, abs(gyro_result));
        //analogWrite(ena2, abs(gyro_result));
      } // else
      else if (gyro_result < 0) { // leaning backwards
        // set direction to backwards
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        /*digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);*/
        
        // make motor move in proportion to distance from target
        analogWrite(ena1, abs(gyro_result));
        //analogWrite(ena2, abs(gyro_result));
      }
    } // end if not tipped over
    // else if it is tipped over, hit the brakes
    else
      brakes();
    
  }
  
  // Print stuff
  if (millis() - print_timePrev > 1000) {
    Serial.print(gyro_read);
    space(gyro_read);
    Serial.println(gyro_disp);
    print_timePrev += 1000;
  }
}



void brakes(){
  digitalWrite(ena1, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  /*digitalWrite(ena2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);*/
}

void space(float num){
  String string = String(num);
  short numSpaces = 8 - string.length();
  String spaces = "";
  for(int i = 0; i < numSpaces; i++){
    spaces += " ";
  }
  Serial.print(spaces);
}


