/*********************************************************

Quad-Wheel Mecanum Vectoring Robot with RC Control
Brent Clay
SuperDroid Robots
December 1, 2015

This code uses an Arduino Uno mounted on a Quad-Wheel Mecanum Vectoring
platform (TP-152-004, TP-095-004, or TP-252-004). The Arduino commands
two dual channel Sabertooth motor controllers to drive four independent
Mecanum wheels. Sabertooths are set to serial mode with addresses 128
and 129.

This firmware allows vectoring RC control of the robot's motion

Code is written for a Spektrum remote + receiver but could easily be reused
for other RC approaches

Platforms:
http://www.superdroidrobots.com/shop/item.aspx/programmable-mecanum-wheel-vectoring-robot-ig52-db/1788/
http://www.superdroidrobots.com/shop/item.aspx/programmable-mecanum-wheel-vectoring-robot-ig32-sb/1713/
http://www.superdroidrobots.com/shop/item.aspx/programmable-mecanum-wheel-vectoring-robot-ig32-dm/1487/

Motor Controller:
http://www.superdroidrobots.com/shop/item.aspx/sabertooth-dual-12a-motor-driver/1155/

Vectoring Robot Support Page:
http://www.superdroidrobots.com/shop/custom.aspx/vectoring-robots/44/

Spektrum DX5e:
http://www.superdroidrobots.com/shop/item.aspx/spektrum-dx5etransmitter-with-ar610-receiver/992/

***********************************************************/

// ****************************************************
// Libraries
// ****************************************************
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// *********************
// Define hardware pins
// *********************
// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 13, drivePinRC = 12, turnPinRC = 11;
int strafeSignal5Vpin = 5;
int eStopPin = 7;
// note: sabertooth pins are 6 for Tx(S1) and 7 for EStop(S2)

// *********************
// RC Vars
// *********************
unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1051, pulseHigh = 1890;

// *********************
// Sabertooth Motor Controllers
// *********************
SoftwareSerial mcSerial(NOT_A_PIN,6);
Sabertooth ST1(128, mcSerial);  //address 128
Sabertooth ST2(129, mcSerial);  //address 129
float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;


void setup() {
  // init sabertooth serial and set autobaud
  mcSerial.begin(9600);  //S1 for sabertooths
  Sabertooth::autobaud(mcSerial);
  delay(500);
  
  // set estop pin as output and pull high
  pinMode(eStopPin,OUTPUT); digitalWrite(eStopPin,HIGH);
  
  // 5V reference for strafe signal
  pinMode(strafeSignal5Vpin, OUTPUT); digitalWrite(strafeSignal5Vpin, HIGH);
  
  // slope/intercept for converting RC signal to range [-1,1]
  mFloat = (float)2 / (pulseHigh - pulseLow);
  bFloat = -1*pulseLow*mFloat;
  
  // slope/intercept for converting [-1,1] to [-127,127]
  mByte = (float)255 / (1 -  0);
  bByte = 0;
  
  Serial.begin(9600);  //debug output
}

void loop() {
  // Read in the RC pulses
  DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);//, PULSEIN_TIMEOUT);
  TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH);//, PULSEIN_TIMEOUT);
  STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH);//, PULSEIN_TIMEOUT);

  // If pulses too short, throw sabertooth estop
  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
    digitalWrite(eStopPin, LOW);
    return;
  }
  
  // otherwise, unthrow estop
  digitalWrite(eStopPin, HIGH);
  
  // convert RC signals to continuous values from [-1,1]
  float driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH);
  float turnVal  = -1*convertRCtoFloat(TURN_PULSE_WIDTH);
  float strafeVal = convertRCtoFloat(STRAFE_PULSE_WIDTH);
  
  // convert the [-1,1] values to bytes in range [-127,127] for sabertooths
  char motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
  char motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
  char motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
  char motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);
  
  // command motors
  ST1.motor(1,motorFL); ST1.motor(2,motorFR);
  ST2.motor(1,motorRR); ST2.motor(2,motorRL);
  
  //mcSerial.print("EOF");  //realterm sync
  
  // debug print
  Serial.print(DRIVE_PULSE_WIDTH); Serial.print(","); Serial.print(TURN_PULSE_WIDTH);
  Serial.print(","); Serial.print(STRAFE_PULSE_WIDTH);
  Serial.print("\t");
  Serial.print(motorFL);  Serial.print(","); Serial.print(motorFR);
  Serial.print("\n");
}

float convertRCtoFloat(unsigned long pulseWidth)
{
  // deadband
  if(pulseWidth > 1450 && pulseWidth < 1550) { pulseWidth = (float)(pulseHigh + pulseLow) / 2; }
  
  float checkVal = mFloat*pulseWidth + bFloat - 1;
  checkVal = checkVal < -1 ? -1 : checkVal;
  checkVal = checkVal >  1 ?  1 : checkVal;
  
  return checkVal;
}

char convertFloatToByte(float value)
{
  float checkVal = mByte*value + bByte;
  checkVal = checkVal < -127 ? -127 : checkVal;
  checkVal = checkVal >  127 ?  127 : checkVal;
  
  char returnVal = (char)(checkVal);
  return returnVal;
}




