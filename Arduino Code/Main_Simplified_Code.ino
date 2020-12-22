/*
Universal Tensile Testing Machine Code By Xieshi aka CrazyBlackStone
VERSION 2.1.1 REV 3
Designed for use with custom PCB
Project published under CC-BY-NC-SA license https://creativecommons.org/licenses/by-nc-sa/4.0/
*/

//Need Wire library. It should come preinstalled with Arduino IDE.
#include <Wire.h>
 
//Need HX711 library https://github.com/bogde/HX711
#include <HX711.h>

//Need U8x8 library, which is included in the u8g2 library https://github.com/olikraus/u8g2
#include <U8x8lib.h>

//----------USER PARAMETERS----------

//PLX-DAQ - Excel data collection. Uncomment this if you have PLX-DAQ set up for data analysis.
//#define plxdaq

//Calibration setup: if you know your load cell's calibration factor, uncomment and change the calibration factor below. If you don't want to calibrate the load cell, comment this out.
#define calibration

#define calibrationFactor 1.9305 * 2280 * 0.09806// this value is obtained by calibrating the scale with known weights, details in https://github.com/bogde/HX711

//set this to the basic steps per revolution of your stepper motor (not counting in gear ratio). It is 200 for 1.8 degree stepper motors and 400 for 0.9 degree stepper motors. Leave unchanged if you don't know - in most cases it is 200 steps per rev.
#define stepsPerRev 200
//set this to the microstepping setting. This does not control the microstepping of the driver - it simply sets the step rate. Set the microstepping mode in hardware through the MS pins.
#define microStep 16
//set this to the gear ratio of the stepper motor you're using
#define gearRatio 100
//set this to the lead screw lead you're using
#define leadScrewPitch 8
//don't change this unless you want to change the modulus start speed
#define modulusSpeedMultiplier 25
//don't change this unless you want to change the slow speed
#define slowSpeedMultiplier 50
//don't change this unless you want to change the fast speed
#define fastSpeedMultiplier 100
//modulus test threshold. Used to determine when to change speed to fast.
#define modulusThreshold 30000
//reading attempts to be taken on each reading; the average of the attempts will be displayed. Depends on your load cell amplifier's configuration. 10hz - 1 attempt, 80hz - 8 attempts.
#define readAttempts 1
 
//pin definitions
#define EStopPin 2
#define tarePin 3
#define moveUpPin 4
#define modePin 5
#define startPin 6
#define moveDownPin 7
#define ledPin 8
#define endStop1Pin 9
#define endStop2Pin 10
#define enablePin 13
#define DTPin A0
#define SCKPin A1
#define dCalDTPin A2
#define dCalSCKPin A3

//parameters
bool testStart = false;
bool moveStepper = false;
byte stepperDir = 0;
byte stepperStatus = 0; //0 = stopped, 1 = moving
int multiplier = 1;
#define modulusSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * modulusSpeedMultiplier
#define slowSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * slowSpeedMultiplier
#define fastSpeed stepsPerRev * microStep * gearRatio / leadScrewPitch / 60 * fastSpeedMultiplier
int stepperSpeed = slowSpeed;
byte stepperSpeedHigh;
byte stepperSpeedLow;
#define slowMeasurementDelay 0.8 * 1000000
#define fastMeasurementDelay 0.1 * 1000000
float measurementDelay = slowMeasurementDelay;
float measurementMax = 0;
unsigned long lastMeasurement = 0;
unsigned long testStartTime = 0;
unsigned long testTime = 0;
byte mode = 1; //1 - tensile slow, 2 - tensile fast, 3 - compression slow, 4 - compression fast, 5 - modulus
bool modeOldState = HIGH;
bool startOldState = HIGH;
bool tareOldState = HIGH;
bool moveUpOldState = HIGH;
bool moveDownOldState = HIGH;
bool confirmMode = false;
bool emergencyStop = false;
String stringMode;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   
HX711 scale;
 
void setup() {
  Serial.begin(9600);
  Wire.begin();
  #ifndef plxdaq
  Serial.println(F("INITIALIZING"));
  #endif

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_inr21_2x4_r);

  u8x8.setCursor(0,0);
  
  scale.begin(DTPin, SCKPin);
 
  #ifdef calibration
  scale.set_scale(calibrationFactor); 
  #endif
 
  scale.tare(20);

  pinMode(moveUpPin, INPUT);
  digitalWrite(moveUpPin, HIGH);

  pinMode(moveDownPin, INPUT);
  digitalWrite(moveDownPin, HIGH);

  pinMode(modePin, INPUT);
  digitalWrite(modePin, HIGH);

  pinMode(startPin, INPUT);
  digitalWrite(startPin, HIGH);
  
  pinMode(tarePin, INPUT);
  digitalWrite(tarePin, HIGH);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  pinMode(EStopPin, INPUT);
  digitalWrite(EStopPin, HIGH);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  
  pinMode(endStop1Pin, INPUT);
  digitalWrite(endStop1Pin, HIGH);

  pinMode(endStop2Pin, INPUT);
  digitalWrite(endStop2Pin, HIGH);

  attachInterrupt(digitalPinToInterrupt(EStopPin), stopNow, FALLING);

  #ifndef plxdaq
  Serial.println(F("INITIALIZATION COMPLETE"));
  #else
  Serial.println(F("CLEARDATA"));
  Serial.println(F("LABEL,Time,Test Time,Value"));
  #endif
}
 
void loop() {

  //EMERGENCY STOP
  if (!digitalRead(endStop1Pin) || !digitalRead(endStop2Pin) && !emergencyStop)
  {
    stopNow();
    Serial.println(F("Endstop triggered - emergency stop"));
  }
  
  if (emergencyStop)
  { 
    stepperSpeed = 0;
    stepperDir = 0;
    stepperStatus = 0;
    sendCommand();
    
    Serial.println(F("\n\n\n-EMERGENCY STOP - RESET TO CLEAR-\n-EMERGENCY STOP - RESET TO CLEAR-\n-EMERGENCY STOP - RESET TO CLEAR-"));
    u8x8.clear();
    u8x8.print(F("E-STOP\nRESET"));
    for (;;);
  }
  
  String inputString;
  bool serialAvailable = false;
  while (Serial.available())
  {
    serialAvailable = true;
    inputString = Serial.readString();
    inputString.toLowerCase();
  }
  if (serialAvailable)
  {
    if (inputString == "start")
    {
      checkMode();
      Serial.println("Current mode is: " + stringMode + ". Confirm? Y/N");
      confirmMode = true;
    }
    else if (inputString == "stop")
    {
      stopTest();
    }
    else if (inputString == "tare")
    {
      tareScale();
    }
    else if (inputString == "up")
    {
      moveUp();
    }
    else if (inputString == "down")
    {
      moveDown();
    }
    else if (inputString == "mode")
    {
      changeMode();
    }
    else if (inputString == "y" && confirmMode)
    {
      startTest();
    }
    else if (inputString == "n" && confirmMode)
    {
      confirmMode = false;
      Serial.println(F("\n-TEST CANCELLED-\n"));
    }
    else
    {
      Serial.println(F("Unknown Command"));
    }
  }
  
  bool modeNewState = digitalRead(modePin);
  bool moveUpNewState = digitalRead(moveUpPin);
  bool moveDownNewState = digitalRead(moveDownPin);
  bool startNewState = digitalRead(startPin);
  bool tareNewState = digitalRead(tarePin);
  
  if (!digitalRead(moveUpPin) && moveUpNewState != moveUpOldState && moveUpNewState == LOW)
  {
    moveUp();
  }
  else if (!digitalRead(moveDownPin) && moveDownNewState != moveDownOldState && moveDownNewState == LOW)
  {
    moveDown();
  }
  else if (!digitalRead(modePin) && modeNewState != modeOldState && modeNewState == LOW && !testStart)
  {
    changeMode();
    delay(10);
  }
  else if (!digitalRead(startPin) && startNewState != startOldState && startNewState == LOW)
  {
   if (!testStart && !moveStepper)
   {
     startTest();
   }
   else
   {
     stopTest();
   }
   delay(10);
  }
  else if (!digitalRead(tarePin) && tareNewState != tareOldState && tareNewState == LOW)
  {
   tareScale();
   delay(10);
  }
 
  modeOldState = modeNewState;
  startOldState = startNewState;
  moveUpOldState = moveUpNewState;
  moveDownOldState = moveDownNewState;
  tareOldState = tareNewState;

  testTime = millis() - testStartTime;

  if ((micros() - lastMeasurement) >= measurementDelay)
  {
    float measurement = scale.get_units(readAttempts) * multiplier;
    String measurementStr = String(measurement);
    #ifndef plxdaq
    Serial.println(measurementStr);
    #endif
    
    if (!testStart)
    {
      String modeStr = String(mode);
      u8x8.clear();
      u8x8.println(measurementStr + "\nMode: " + modeStr);
    }
    else
    {
      if (measurement >= measurementMax)
      {
        measurementMax = measurement;
      }
      #ifdef plxdaq
      String testTimeStr = String(testTime);
      String millisStr = String(millis());
      Serial.println("DATA,TIME," + millisStr + "," + testTimeStr + "," + measurementStr);
      #endif
    }
    lastMeasurement = micros();
  }

  if (mode == 5 && testTime >= modulusThreshold && stepperSpeed != fastSpeed)
  {
    Serial.println(F("THRESHOLD REACHED"));
    stepperSpeed = fastSpeed;
    stepperDir = 0;
    multiplier = 1;
    stepperStatus = 1;
    sendCommand();
  }
}
 
 
void startTest()
{
  confirmMode = false;
  moveStepper = true;
  //digitalWrite(enablePin, LOW);

  Serial.println(F("\n-TEST START-\n"));

  scale.tare(20);
  measurementDelay = fastMeasurementDelay;
  if (mode == 1)
  {
    stepperSpeed = slowSpeed;
    stepperDir = 0;
    multiplier = 1;
  }
  else if (mode == 2)
  {
    stepperSpeed = fastSpeed;
    stepperDir = 0;
    multiplier = 1;
  }
  else if (mode == 3)
  {
    stepperSpeed = slowSpeed;
    stepperDir = 1;
    multiplier = -1;
  }
  else if (mode == 4)
  {
    stepperSpeed = fastSpeed;
    stepperDir = 1;
    multiplier = -1;
  }
  else if (mode == 5)
  {
    stepperSpeed = modulusSpeed;
    stepperDir = 0;
    multiplier = 1;
  }

  stepperStatus = 1;
  sendCommand();
  
  testStart = true;
  measurementMax = 0;
  testStartTime = millis();

  u8x8.clear();
  u8x8.println("Test\nStarted");
}
 
void stopTest()
{
  Serial.println("\n-STOP-\n");
  measurementDelay = slowMeasurementDelay;

  stepperSpeed = 0;
  stepperDir = 0;
  stepperStatus = 0;
  sendCommand();
  
  moveStepper = false;
  //digitalWrite(enablePin, HIGH);
  
  if (testStart)
  {
    u8x8.clear();
    String measMaxStr = String(measurementMax);
    u8x8.println("Max: \n" + measMaxStr);
    #ifndef plxdaq
    String testTimeStr = String(testTime);
    Serial.println("Time: " + testTimeStr + "\nMaximum value: " + measMaxStr); 
    #endif
    testStart = false;
    delay(10000);
  }
}

void tareScale()
{
  Serial.println(F("\n-SCALE TARE-\n"));   
  scale.tare(20);
}

void moveUp()
{
  moveStepper = true;
  //digitalWrite(enablePin, LOW);
  Serial.println(F("\n-MOVING UP-\n"));

  stepperSpeed = fastSpeed;
  stepperDir = 0;
  stepperStatus = 1;
  sendCommand();
}

void moveDown()
{
  moveStepper = true;
  //digitalWrite(enablePin, LOW);
  Serial.println(F("\n-MOVING DOWN-\n"));

  stepperSpeed = fastSpeed;
  stepperDir = 1;
  stepperStatus = 1;
  sendCommand();
}

void changeMode()
{
  if (mode < 5)
  {
    mode++;
  }
  else
  {
    mode = 1;
  }
  checkMode();
  Serial.println("CURRENT MODE: " + stringMode);
}

void checkMode()
{
  if (mode == 1)
  {
    stringMode = F("SLOW Tensile Test");
  }
  else if (mode == 2)
  {
    stringMode = F("FAST Tensile Test");
  }
  else if (mode == 3)
  {
    stringMode = F("SLOW Compression Test");
  }
  else if (mode == 4)
  {
    stringMode = F("FAST Compression Test");
  }
  else if (mode == 5)
  {
    stringMode = F("Tensile Modulus Test");
  }
}

void stopNow()
{
  digitalWrite(enablePin, HIGH);
  //pinMode(EStopPin, OUTPUT);
  //digitalWrite(EStopPin, LOW);
  
  moveStepper = false;
  emergencyStop = true;
}

void sendCommand()
{
  stepperSpeedHigh = *((uint8_t*)&(stepperSpeed)+1);
  stepperSpeedLow = *((uint8_t*)&(stepperSpeed)+0);
  byte sendInfo[4] = {stepperSpeedHigh, stepperSpeedLow, stepperDir, stepperStatus};
  Wire.beginTransmission(9);
  for (int i=0; i<4; i++)
  {
    Wire.write(sendInfo[i]);  //data bytes are queued in local buffer
  }
  byte sendStatus = Wire.endTransmission();
  if (sendStatus != 0)
  {
    String sendStatusStr = String(sendStatus);
    Serial.println("Send failed (" + sendStatusStr + ")- emergency stop");
    stopNow();
  }
}
