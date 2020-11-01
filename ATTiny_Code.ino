/*
Universal Tensile Testing Machine ATTiny85 Code By Xieshi aka CrazyBlackStone
VERSION 2.1.1 REV 2
Designed for use with custom PCB
Project published under CC-BY-NC-SA license https://creativecommons.org/licenses/by-nc-sa/4.0/
Code published under GNU General Public V3 https://www.gnu.org/licenses/gpl-3.0.en.html
*/

//Need TinWireS library https://github.com/nadavmatalon/TinyWireS
#include <TinyWireS.h>

//pin definitions
const int stepPin = PB3;
const int dirPin = PB4;

//parameters
bool moveStepper;
unsigned long lastStep = 0;
int stepperSpeed = 1000;
int dataArray[3];

void setup() {
  TinyWireS.begin(9);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);
}

void loop() {
  while (TinyWireS.available())
  {
    for(int i=0; i<4; i++)
    {
      dataArray[i] = TinyWireS.receive();
    }
    stepperSpeed = ((dataArray[0]<<8)+dataArray[1])/10;
    if (dataArray[2] == 0)
    {
      digitalWrite(dirPin, LOW);
    }
    else if (dataArray[2] == 1)
    {
      digitalWrite(dirPin, HIGH);
    }
    if (dataArray[3] == 0)
    {
      moveStepper = false;
    }
    else if (dataArray[3] == 1)
    {
      moveStepper = true;
    }
  }
  if (moveStepper)
  {
    if ((micros() - lastStep) >= 1000000. / stepperSpeed)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(5); 
      lastStep = micros();
    }
  }
  TinyWireS_stop_check();
}
