
// Arduino 101 on the Bot as Master
// uses I2C to request and receive data from the Feather M0

#include <Wire.h> // for I2C
#include <CurieIMU.h>


/*********** PID constants ***********/
// Bot stuff                one           two  batteries
const int gyro_prop = 15; //      30            15     
const float gyro_diff = 1.25; //  2 (40/20)     1.25 (25/20)
const int vari_prop = 2; //       5             2
const int vari_diff = 2; //       5             2
const int disp_prop = 60; //      30?           60    (x + 1)
const float disp_diff = 1.1; //   1?            1.1   (1 + x / 100)


/*********** I2C stuff ***********/
// receives this over I2C
// speed, sign, steer, sign, reset
uint8_t dataRx[5] = {99, 88, 77, 66, 55};
// holds the original readings
int data[] = {0, 0, 0};

unsigned long receive_timePrev = 0;
const unsigned int receive_interval = 100; // 10 times per second


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
byte gyro_buffer = 35; // gyro_limit - gyro_buffer, area inside of gyro_limit to cap tilt within
                                                 // see displace stuff in loop


/*********** Gyro stuff ***********/
const byte gyro_limit = 45; // stop working outside of this area
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
unsigned long print_timePrev;
unsigned long printTime;
unsigned long loopTime;


/*********** notification led ***********/
// To tell when bot is done calibrating.
// Will blink at 1 second intervals.
byte blinkState = 0; // off to begin with
unsigned long blinkPrevTime;
const unsigned int blinkDelay = 500;


void setup() {
  Serial.begin(19200);
  //while (!Serial) ; // wait until serial monitor is open

/*********** Gyro stuff ***********/
  CurieIMU.begin();
  CurieIMU.setGyroRate(updateRate);
  // set gyro to lowest range for maximum accuracy!
  CurieIMU.setGyroRange(range);
  
  // calibrate gyro
  Serial.println("Calibrating");
  float gyro_temp = 0;
  int polls = 1, targetPolls = 300; // starts at 1 so targetPolls is actual count
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

/*********** I2C stuff ***********/  
  Wire.begin(9);                // join i2c bus with address #9
  receive_timePrev = millis();

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
  attachInterrupt( digitalPinToInterrupt(chA1), channelA1r, RISING );
  attachInterrupt( digitalPinToInterrupt(chB1), channelB1r, RISING );
  
  // motor 2
  pinMode(chA2, INPUT);
  pinMode(chB2, INPUT);
  attachInterrupt( digitalPinToInterrupt(chA2), channelA2r, RISING );
  attachInterrupt( digitalPinToInterrupt(chB2), channelB2r, RISING );

/*********** Print stuff ***********/
  print_timePrev = millis();
  printTime = millis();
  loopTime = micros(); // to measure the balance loop in microseconds

/*********** notification led ***********/
  blinkPrevTime = millis();
  pinMode(13, OUTPUT);
}


void loop() {
  
/*********** notification led ***********/
  if (millis() - blinkPrevTime > blinkDelay) {
    if (blinkState) {  // if currently on, turn off
      digitalWrite(13, LOW);
      blinkState = 0;
    } else {  // led currently off, so turn on
      digitalWrite(13, HIGH);
      blinkState = 1;
    }
    blinkPrevTime += blinkDelay;
  }  // end notification led blink loop
  
/*********** I2C stuff ***********/
  if (millis() - receive_timePrev > receive_interval) { // 10 times a second
    receive_timePrev += receive_interval;
    
    // update RC readings
    //Serial.print("requesting readings, ");
    Wire.requestFrom(8, 5);    // from slave device #8, request 5 bytes
    
    if (Wire.available()){       // check if we've received something
      byte count = 0;
      //Serial.print("receiving readings, ");
      while (Wire.available()) { // slave may send less than requested, or nothing at all?
        dataRx[count++] = (uint8_t)Wire.read();    // receive each byte; must cast because was sent as char*
      }

      // convert displacement (forwards/backwards)
      data[0] = map(dataRx[0], 0, 255, 0, 7);
      // see if it was negative
      if (dataRx[1])
        data[0] = -data[0];

      // convert variance (steering left/right)
      data[1] = map(dataRx[2], 0, 255, 0, 3);
      // see if it was negative
      if (dataRx[3])
        data[1] = -data[1];

      // get the reset value
      data[2] = dataRx[4];
      
    } // end if Wire available

    printStuff();

    // manipulate the encoders in the balance loop because that runs faster,
    // giving more resolution and hopefully preventing the jittering!
    
  } // end I2C loop


// balance time loop
  if (millis() - gyro_timePrev > updateTime) {
    //loopTime = micros();

/*********** zeroing out gyroscope ***********/
    int maxError = gyro_limit - gyro_buffer;
    if (gyro_disp < 0) {
      gyro_disp += .1;
    } else if (gyro_disp > 0) {
      gyro_disp -= .1;
    }
    

/*********** I2C Encoder manipulation ***********/
                    // displace
                            // steering; make them do opposites so the bot twists
    enco1 = enco1 - data[0] + data[1];
    enco2 = enco2 - data[0] - data[1];

/*********** Displace stuff ***********/
    // The variance PID part should guarantee that the motors
    // are at roughly the same position. However, it feels more
    // "balanced" using the average of the motor positions.

    // enco1 & 2 should focus around 0.
    disp_avg = round( (enco1 + enco2) / 2 ); // probs compute intensive but whatevs
  //Serial.print(disp_avg); // distance
  //Serial.print(' ');
    disp_change = disp_avg - disp_prev;
  //Serial.print(disp_change); // speed
  //Serial.print(' ');
    disp_prev = disp_avg; // reset prev

    // If the bot is far forward, we want it to try and lean backward.
    // Thus, the negative multiplier.
                        // prop                       // diff
    gyro_target = -( (float)disp_avg / disp_prop + disp_change * disp_diff );

    /* cap the gyro target at either end*/
    // if the target is less than the limit plus a small buffer,
    if (gyro_target < -gyro_limit + gyro_buffer) {
      // set it to that
      gyro_target = -gyro_limit + gyro_buffer;
    } // else
    // if the target is greater than the limit minus the small buffer
    else if (gyro_target > gyro_limit - gyro_buffer) {
      // set it to that
      gyro_target = gyro_limit - gyro_buffer;
    }/**/
  //Serial.print(gyro_target); // result
  //Serial.print(' ');
  
/*********** Gyro stuff ***********/
    gyro_timePrev += updateTime;
    gyro_read = CurieIMU.readGyroScaled(axis) - gyro_calib;
    gyro_disp += gyro_read / updateRate;
  //Serial.print(gyro_disp); // distance
  //Serial.print(' ');
  //Serial.print(gyro_read); // speed
  //Serial.print(' ');
    
    //gyro_target = 0;
    // Without target, it was essentially "gyro_disp - 0"
    // So replace '0' with 'target'.
    gyro_error = gyro_disp - gyro_target;

    // If leaning forward, will be rotating backwards, so
    // # + (-#)    or    (-#) + #
    gyro_result = gyro_error * gyro_prop + gyro_read * gyro_diff;

    // set the motors to the gyro
    motor1_result = motor2_result = gyro_result;
  //Serial.print(motor1_result); // result
  //Serial.print(' ');
    
    
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
    if (gyro_disp > -gyro_limit && gyro_disp < gyro_limit) {
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
      // motor 2 backwards
      else if (motor2_result < 0) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena2, abs(motor2_result));
        //Serial.print("2 backwards");
      }
    } // end if not tipped over
    
    else // if it is tipped over, hit the brakes
      brakes();

    //printStuff();
    //Serial.println();
    //Serial.println(micros() - loopTime); // takes just 50 microseconds :O
  } // end gyro update loop

  /*int deley = 1000000;
  if (micros() - print_timePrev > deley) {
    printStuff();
    print_timePrev += deley;
    Serial.println();
  }*/

  // zero out the gyro displacement if I send serial data to it
  // or data[4] is high, i.e. the thumbstick is being held
  if (data[2] || Serial.available() > 0) {
    gyro_disp = 0;
    enco1 = 0;
    enco2 = 0;
    Serial.read(); // to clear the Serial input and make it unavailable
  }
  
}



void printStuff(){
  //printTime = micros();
   
/*********** I2C stuff ***********
  Serial.print(data[0]);
  space(data[0]);
  Serial.print(data[1]);
  space(data[1]);
  Serial.print(data[2]);
  space(data[2]);
    
/*********** Displace stuff ***********
  //Serial.println();
  Serial.print(disp_avg); // distance
  Serial.print(' ');
  //space(disp_avg);
  //Serial.print(disp_prop);
  //space(disp_prop);
  Serial.print(disp_change); // speed
  Serial.print(' ');
  //space(disp_change);
  //Serial.print(disp_diff);
  //space(disp_diff);
  Serial.print(gyro_target); // result
  Serial.print(' ');
  //space(gyro_target);
    
/*********** Gyro stuff ***********/
  //Serial.println();
  Serial.print(gyro_disp); // distance
  //Serial.print(' ');
  space(gyro_disp);
  //Serial.print(gyro_prop);
  //space(gyro_prop);
  Serial.print(gyro_read); // speed
  //Serial.print(' ');
  space(gyro_read);
  //Serial.print(gyro_diff);
  //space(gyro_diff);
  //Serial.print(motor1_result); // result
  //Serial.print(' ');
  //space(motor1_result);
    
/*********** Variance stuff ***********/
  //Serial.println();
  Serial.print(variance * vari_prop - vari_change * vari_diff);
  //Serial.print(' ');
  space(variance * vari_prop - vari_change * vari_diff);
  Serial.print(variance);
  //Serial.print(' ');
  space(variance);
  //Serial.print(vari_prop);
  //space(disp_prop);
  //Serial.print(vari_change);
  //space(disp_change);
  //Serial.print(vari_diff);
  //space(disp_diff);

/*********** Encoder stuff ***********/
  Serial.print(enco1);
  space(enco1);
  Serial.print(enco2);
  space(enco1);

/**/
//Serial.println(micros() - printTime); // takes 2 milliseconds!!!
Serial.println();
} // end printStuff()

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

void space(float num){
  String string = String(num);
  short numSpaces = 12 - string.length();
  String spaces = "";
  for(int i = 0; i < numSpaces; i++){
    spaces += " ";
  }
  Serial.print(spaces);
}


