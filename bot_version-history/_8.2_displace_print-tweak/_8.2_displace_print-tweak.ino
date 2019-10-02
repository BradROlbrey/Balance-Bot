
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
BLEIntCharacteristic Vari_Prop("3E099913-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Vari_Diff("3E099914-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Prop("3E099915-294F-11E5-93BE-AFD1FE6D1DFD", props);
BLEIntCharacteristic Disp_Diff("3E099916-294F-11E5-93BE-AFD1FE6D1DFD", props);

// Bot stuff                one           two  batteries
int gyro_prop = 30; //      30            15     
float gyro_diff = 1.25; //  2 (40/20)     1.25 (25/20)
int vari_prop = 2; //       5             2
int vari_diff = 2; //       5             2
int disp_prop = 30; //      30?           60    (x + 1)
float disp_diff = 1.1; //   1?            1.1   (1 + x / 100)



/*********** Motor stuff ***********/
// motor 1
const byte in1 = 4; // green;   out1 = orange
const byte in2 = 2; // blue;    out2 = yellow
const byte ena1 = 3; // yellow

// motor 2
const byte in3 = 7; // green    out3 = orange
const byte in4 = 5; // blue     out4 = yellow
const byte ena2 = 6; // yellow

// power inputs
float motor1_result; // = gyros + variance;
float motor2_result;

/*********** Encoder stuff ***********/
// motor 1
const byte chA1 = 8; // green
const byte chB1 = 9; // white
volatile long enco1 = 0;
const uint32_t ioReg1=SS_GPIO_8B1_BASE_ADDR+SS_GPIO_EXT_PORTA; // for some random pins
#define readPin8 (__builtin_arc_lr(ioReg1)&0b01000000)
#define readPin9 (__builtin_arc_lr(ioReg1)&0b00100000)

// motor 2
const byte chA2 = 10; // green
const byte chB2 = 11; // white
volatile long enco2 = 0;
const uint32_t ioReg2=SOC_GPIO_BASE_ADDR+SOC_GPIO_EXT_PORTA; // for some other random pins
#define readPin10 (MMIO_REG_VAL(ioReg2)&0b100000000000)
#define readPin11 (MMIO_REG_VAL(ioReg2)&0b10000000000)


/*********** Displace stuff ***********/
int disp_avg = 0; // average of the rotations of the motors.
int disp_prev = 0;
int disp_change = 0;
// padding between gyro target and gyro limit
byte gyro_buffer = 10;


/*********** Gyro stuff ***********/
const byte gyro_limit = 25;
const int updateRate = 100; // update frequency: x times per second
const int updateTime = 1000/updateRate; // ms per update
unsigned long gyro_timePrev = 0;
const int range = 250; // deg/sec
const byte axis = 0;
float gyro_read = 0;   // reading of gyro each loop
float gyro_calib = 0;  // offset 
float gyro_disp = 0;   // accumulated displacement
float gyro_target = 0; // where the gyro wants the bot to be; affected by displacement
float gyro_error = 0;  // difference between the displacement and the error
float gyro_result = 0; // difference of target and disp, multiplied by prop


/*********** Variance stuff ***********/
// variance is the difference in the rotations of the motors
int variance = 0;
int vari_prev = 0;
int vari_change = 0;


/*********** Print stuff ***********/
unsigned long print_timePrev = 0;




void setup() {
  Serial.begin(9600);
  while (!Serial) ; // wait until serial monitor is open

/*********** Gyro stuff ***********/
  CurieIMU.begin();
  CurieIMU.setGyroRate(updateRate);
  // set gyro to lowest range for maximum accuracy!
  CurieIMU.setGyroRange(range);
  
  // calibrate gyro
  Serial.println("Calibrating");
  float gyro_temp = 0;
  int polls = 1, targetPolls = 300;
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
  // motor 2
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena2, OUTPUT);
  // start braked
  brakes();

/*********** Encoder stuff ***********/
  // motor 1
  pinMode(chA1, INPUT);
  pinMode(chB1, INPUT);
  attachInterrupt( chA1, channelA1r, RISING );
  attachInterrupt( chB1, channelB1r, RISING );
  
  // motor 2
  pinMode(chA2, INPUT);
  pinMode(chB2, INPUT);
  attachInterrupt( chA2, channelA2r, RISING );
  attachInterrupt( chB2, channelB2r, RISING );


/*********** Bluetooth stuff ***********/
  // Bluetooth
  // set advertising packet content
  blePeripheral.setLocalName(device_name);
  blePeripheral.setAdvertisedServiceUuid(Bot.uuid());

  // add services and characteristics
  blePeripheral.addAttribute(Bot);
  blePeripheral.addAttribute(Gyro_Prop);
  blePeripheral.addAttribute(Gyro_Diff);
  blePeripheral.addAttribute(Vari_Prop);
  blePeripheral.addAttribute(Vari_Diff);
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
      gyro_diff = (float) Gyro_Diff.value() / 20.0;
    if (Vari_Prop.written())
      vari_prop = Vari_Prop.value();
    if (Vari_Diff.written())
      vari_diff = Vari_Diff.value();
    if (Disp_Prop.written())
      disp_prop = Disp_Prop.value() + 1;
    if (Disp_Diff.written())
      disp_diff = (float) Disp_Diff.value() / 100 + 1;
    
  } // else, we are not supposed to be here
  //else Serial.print("Bluetooth problem");


// balance time loop
  if (millis() - gyro_timePrev > updateTime) {
    
/*********** Displace stuff ***********/
    // The variance PID part should guarantee that the motors
    // are at roughly the same position. However, it feels more
    // "balanced" using the average of the motor positions.

    // enco1 & 2 should focus around 0.
    disp_avg = round( (enco1 + enco2) / 2 );
    disp_change = disp_avg - disp_prev;
    disp_prev = disp_avg; // reset prev

    // If the bot is far forward, we want it to try and lean backward.
    // Thus, the negative multiplier.
                  // prop                             // diff
    gyro_target = -1* ( (float)disp_avg / disp_prop + disp_change * disp_diff );

    /* cap the gyro target at either end*/
    // if the target is less than the limit plus a small buffer,
    if (gyro_target < -1*gyro_limit + gyro_buffer) {
      // set it to that
      gyro_target = -1*gyro_limit + gyro_buffer;
    } // else
    // if the target is greater than the limit minus the small buffer
    else if (gyro_target > gyro_limit - gyro_buffer) {
      // set it to that
      gyro_target = gyro_limit - gyro_buffer;
    }/**/
    
/*********** Gyro stuff ***********/
    gyro_timePrev += updateTime;
    gyro_read = CurieIMU.readGyroScaled(axis) - gyro_calib;
    gyro_disp += gyro_read / updateRate;
    
    //gyro_target = 0;
    // Without target, it was essentially "gyro_disp - 0"
    // So replace '0' with 'target'.
    gyro_error = gyro_disp - gyro_target;

    // If leaning forward, will be rotating backwards, so
    // # + (-#)    or    (-#) + #
    gyro_result = gyro_error * gyro_prop + gyro_read * gyro_diff;

    // set the motors to the gyro
    motor1_result = motor2_result = gyro_result;
    
    
/*********** Variance stuff ***********/
    variance = enco1 - enco2;
    vari_change = variance - vari_prev;
    
    // if positive, motor1 is ahead of motor 2, so
    if (variance > 0) {
      // subtract some from motor1
      motor1_result -= variance * vari_prop;
      // add some to motor 2
      motor2_result += variance * vari_prop;
    }
    else { // if negative, motor 1 is behind motor 2; doesn't matter if it's equal
      // subtract a negative, adding some to motor1
      motor1_result -= variance * vari_prop;
      // add a negative, subtracting some from motor 2
      motor2_result += variance * vari_prop;
    }

     
    motor1_result -= vari_change * vari_diff;
    motor2_result -= vari_change * vari_diff;
    //motor1_result = motor2_result = 0;
    
    vari_prev = variance;
    
/*********** Motor PID stuff ***********/

    // if the bot is not completely tipped over
    if (gyro_disp > -1*gyro_limit && gyro_disp < gyro_limit) {
      //Serial.print("Upright");
      // motor 1 forwards
      if (motor1_result > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena1, abs(motor1_result));
        //Serial.print("1 forwards");
      }
      // motor 1 backwards
      else if (motor1_result < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(ena1, abs(motor1_result));
        //Serial.print("1 backwards");
      }

      // motor 2 forwards
      if (motor2_result > 0) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena2, abs(motor2_result));
        //Serial.print("2 forwards");
      }
      // motor 2 forwards
      else if (motor2_result < 0) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena2, abs(motor2_result));
        //Serial.print("2 backwards");
      }
    } // end if not tipped over
    // else if it is tipped over, hit the brakes
    else
      brakes();
    
  }

  printStuff();

  // zero out the gyro displacement if I send serial data to it
  if (Serial.available() > 0) {
    gyro_disp = 0;
    enco1 = 0;
    enco2 = 0;
    Serial.read(); // to clear the Serial input and make it unavailable
  }
}


void printStuff(){
  int deley = 100;
  if (millis() - print_timePrev > deley) {
    
/*********** Gyro stuff ***********/
    //Serial.println();
    Serial.print(motor1_result);
    space(motor1_result);
    Serial.print(gyro_disp);
    space(gyro_disp);
    Serial.print(gyro_prop);
    space(gyro_prop);
    Serial.print(gyro_read);
    space(gyro_read);
    Serial.print(gyro_diff);
    space(gyro_diff);
    
/*********** Variance stuff ***********
    //Serial.println();
    Serial.print(variance * vari_prop - vari_change * vari_diff);
    space(gyro_target);
    Serial.print(variance);
    space(disp_avg);
    Serial.print(vari_prop);
    space(disp_prop);
    Serial.print(vari_change);
    space(disp_change);
    Serial.print(vari_diff);
    space(disp_diff);
    
/*********** Displace stuff ***********
    //Serial.println();
    Serial.print(gyro_target);
    space(gyro_target);
    Serial.print(disp_avg);
    space(disp_avg);
    Serial.print(disp_prop);
    space(disp_prop);
    Serial.print(disp_change);
    space(disp_change);
    Serial.print(disp_diff);
    space(disp_diff);

/**/
    print_timePrev += deley;
    Serial.println();
  }
}

// motor 1
void channelA1r() { // if channel A went from low to high
  if (readPin9) // if channel B is high, we went forwards, I guess
    enco1++;
  else // else channel B is low, and we went backwards, supposedly
    enco1--;
  //Serial.println("A");
}
void channelB1r() { // if channel B went from low to high
  if (readPin8) // if channel A is high, we went backwards
    enco1--;
  else // else channel A is low, and we went forwards
    enco1++;
  //Serial.println("B");
}

// motor 2
void channelA2r() { // if channel A went from low to high
  if (readPin11) // if channel B is high, we went backwards
    enco2--;
  else // else channel B is low, and we went forwards
    enco2++;
  //Serial.println("A");
}
void channelB2r() { // if channel B went from low to high
  if (readPin10) // if channel A is high, we went forwards
    enco2++;
  else // else channel A is low, and we went backwards
    enco2--;
  //Serial.println("B");
}

void brakes(){
  digitalWrite(ena1, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(ena2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void space(float num){
  String string = String(num);
  short numSpaces = 12 - string.length();
  String spaces = "";
  for(int i = 0; i < numSpaces; i++){
    spaces += " ";
  }
  Serial.print(spaces);
}


