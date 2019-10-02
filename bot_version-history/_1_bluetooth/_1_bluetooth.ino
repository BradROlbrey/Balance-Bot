
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

void setup() {
  Serial.begin(9600);
  while(!Serial) ; // wait until serial monitor is open

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
  else Serial.print("Bluetooth problem");


  
  // bot stuff

  while (millis() - time_prev > 1000) {
    Serial.print(gyro_prop);
    spaces(gyro_prop);
    Serial.print(gyro_diff);
    spaces(gyro_diff);
    Serial.print(disp_prop);
    spaces(disp_prop);
    Serial.println(disp_diff);
    time_prev += 1000;
  }
  
  delay(1000);

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


