
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
int disp_prop = 5; // proportional of displacement
int disp_diff = 100; // differential of displacement

/*********** PID and motor and encoder stuff ***********/

// motors
const byte in1 = 2; // green;     out1 = orange
const byte in2 = 4; // blue;      out2 = yellow
const byte ena = 3; // yellow

// encoders
const byte chA = 8; // green
const byte chB = 9; // white
volatile long revs = 0;
uint32_t ioReg1=SS_GPIO_8B1_BASE_ADDR+SS_GPIO_EXT_PORTA; // for some random pins
#define readPin8 (__builtin_arc_lr(ioReg1)&0b01000000)
#define readPin9 (__builtin_arc_lr(ioReg1)&0b00100000)

// proportional
long target = 0;
int disp_error = 0;
const byte tolerance = 0;
// differential
int disp_changeRate = 0; // rate of change of disp_error
double revs_prev = 0; // for getting revol difference between updates
unsigned long time_prev_speed = 0; // for getting rate of revol changes
//result
int result = 0;



void setup() {
  Serial.begin(9600);
  while(!Serial) ; // wait until serial monitor is open

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

  /*********** PID and motor and encoder stuff ***********/

  // motors; start braked & not moving
  pinMode(in1, OUTPUT);
  digitalWrite(in1, LOW);
  pinMode(in2, OUTPUT);
  digitalWrite(in2, LOW);
  pinMode(ena, OUTPUT);
  digitalWrite(ena, LOW);

  // encoders
  pinMode(chA, INPUT);
  pinMode(chB, INPUT);
  attachInterrupt( chA, channelAr, RISING );
  attachInterrupt( chB, channelBr, RISING );

}

// serial print delay
unsigned long time_prev = 0;

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
  else Serial.print("Bluetooth problem");

  /*********** PID and motor and encoder stuff ***********/
  disp_changeRate = revs - revs_prev;
  revs_prev = revs;
  disp_error = target - revs; // distance to target
  
  result = disp_error * disp_prop - disp_changeRate * disp_diff;
  // take the absolute value only after all the math!
  // if reverse direction only by disp_error, abs val of result means the motor just keeps spinning
  if (result > tolerance) { // target greater than current pos. We are behind target, 
    // set direction to forwards
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    // make motor move in proportion to distance from target
    // subtract positive movement of motor
    analogWrite(ena, abs(result));
  } // else
  else if (result < -1*tolerance) { // target less than current pos. We are in front of target,
    // set direction to backwards
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    
    // make motor move in proportion to distance from target
    // subtract negative movement of motor
    analogWrite(ena, abs(result));
  }
  else{ // if we are within the tolerance, at an appropriate postion...
    // full brakes
    digitalWrite(ena, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }



  while (millis() - time_prev > 1000) {
    /*Serial.print(gyro_prop);
    spaces(gyro_prop);
    Serial.print(gyro_diff);
    spaces(gyro_diff);
    Serial.print(disp_prop);
    spaces(disp_prop);
    Serial.println(disp_diff);*/

    Serial.print(result);

    Serial.println();
    time_prev += 1000;
  }

}



void channelAr() { // if channel A went from low to high
  if (readPin9) // if channel B is high, we went backwards
    revs--;
  else // else channel B is low, and we went forwards
    revs++;
  //Serial.println("A");
}
void channelBr() { // if channel B went from low to high
  if (readPin8) // if channel A is high, we went forwards
    revs++;
  else // else channel A is low, and we went backwards
    revs--;
  //Serial.println("B");
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


