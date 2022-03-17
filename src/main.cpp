#include "LetoBldc.h"
#include "writeCalibtationData.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>

#define targetSpeed 158
LETO_BLDC_Motor Motor;
//LETO_BLDC_Motor hAxialMotor;

uint8_t serialBitCounter = 0;
//u_int16_t ScanStartPos_H = 6000;
u_int16_t TargetLocation_1= 3000;
u_int16_t TargetLocation_2= 6000;
bool TestButton = false;
bool NewMotor = false;
char receivedSerialCMD[10];

bool CheckPID();
bool CheckEndStop();
bool MechRange();
bool CheckTemp();
bool CheckProtection();
bool IsMotorMoving();
bool PowerOnSleep();
void MotorInitial();
void processSerialCMD();

void setup()
{
  Serial.begin(115200);
  //hAxialMotor.I2C_addr = 0x50 >> 1;
  Motor.I2C_addr = 0x52 >> 1;
  Motor.begin();
  MotorInitial();
  Serial.printf("Moving to target location: %d\r\n", TargetLocation_1);
  Motor.gotoAbsoluteLocationAtSpeed(TargetLocation_1, 2 * targetSpeed);
  delay(200);
  while (Motor.isMotorMoving() != 0)
  {
    delay(1000);
  }
  Serial.printf("Encoder Reading: v: %d\r\n",
                Motor.getEncoderReading());
  Serial.println("Ready for test, send \"$R\" command to start test");
}
void loop()
{
  processSerialCMD();
    if (TestButton == true)
    {
      Serial.println("Motor check and test:");
      if(PowerOnSleep()&&IsMotorMoving()&&CheckPID()&&MechRange()&&CheckTemp()&&CheckProtection())
      {
         Serial.println("Motor is ready to assemble.");
      }
      else
      {
         Serial.println("Something wrong with Motor.");
      }
      TestButton = false;
    }
}

bool CheckPID()
{
    //Check PID
    if(Motor.get_P_Gain() == 1019 && Motor.get_I_Gain() == 5 && Motor.get_D_Gain() == 1024)
    {
        Serial.println("Motor PID is checked");
        return true;
    }
    else
    {
        Serial.printf("Motor PID error: %d, %d, %d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
        return false;
    }
}
bool CheckEndStop()
{
    if(Motor.getFirstEndstop() == 100)
    {
        Serial.println("Endstop is checked");
        return true;
    }
    else
    {
        Serial.printf("Endstop error: %d\r\n", Motor.getFirstEndstop());
        return false;
    }
}
bool CheckTemp()
{
    if(Motor.getTemperature()<50)
    {
        Serial.println("Temperature is checked");
        return true;
    }
    else
    {
        Serial.printf("Over temperature: %d\n\r", Motor.getTemperature());
        return false;
    }
   
}
bool CheckProtection()
{
  if(Motor.getTempProtection() == 0)
  {
    return false;
  }
  else
  {
    return true;
  }
  //hAxialMotor.getTempProtection();  
}
bool IsMotorMoving()
{   
    Motor.gotoAbsoluteLocationAtSpeed(TargetLocation_2, 2 * targetSpeed);
    if(TargetLocation_2 < 5000)
    {
      TargetLocation_2 = TargetLocation_2 + 1000;
    }
    else
    {
      TargetLocation_2 = TargetLocation_2 -3000;
    }
    //Serial.printf("TargetLocation: %d\r\n",TargetLocation_2);
    if(Motor.isMotorMoving() == 0)
    {
        Serial.println("Motor is not moving.");
        return false;
    }
    else
    {
        Serial.println("Motor's move is checked");
        return true;
    }
}
bool PowerOnSleep()
{
  if(Motor.getSleepOnPowerUpMode())
  {
    Serial.println("Sleeping error");
    return false;
  }
  else
  {
    Serial.println("Power on sleep is checked");
    return true;
  }
}
bool MechRange()
{
  if(Motor.getMechanicalRange() == 5100)
  {
    Serial.println("Mechanical range is checked");
    return true;
  }
  else
  {
    Serial.printf("Mechnical range error: %d\r\n", Motor.getMechanicalRange());
    return false;
  }
}
void MotorInitial()
{
  delay(5000);
  Serial.println("Initializing motor:");
  Motor.set_P_Gain(1019);
  Motor.set_I_Gain(5);
  Motor.set_D_Gain(1024);
  if(Motor.get_P_Gain() && Motor.get_I_Gain() && Motor.get_D_Gain())
  {
    Serial.println("I2C is ON");
  }
  else
  {
    Serial.println("I2C is OFF");
  }
  Serial.printf("Motor PID: %d, %d, %d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
  
  Motor.setFirstEndstop(100);
  Serial.printf("Motor endstop: %d\r\n",Motor.getFirstEndstop());
  Motor.setMechanicalRange(5100);
  Motor.getMechanicalRange();
  Motor.setSleepOnPowerUpMode(false);
  Motor.setTempProtection(true);
  delay(2000);


  bool waitHoming = false;
  while (!waitHoming)
  {
    waitHoming = Motor.finishedHoming();
    delay(100);
  }
  delay(2000);
  Serial.println("Initialization is finished.");

}
void processSerialCMD()
{
  while (Serial.available())
  {
    char tempRead = Serial.read();
    if (tempRead == '$')
    {
      serialBitCounter = 0;
      tempRead = Serial.read();
    }
    if (tempRead != '\r' && tempRead != '\n')
    {
      receivedSerialCMD[serialBitCounter++] = tempRead;
      if (serialBitCounter > 9)
      {
        serialBitCounter = 0;
      }
    }
    else if (tempRead == '\r')
    {
      receivedSerialCMD[serialBitCounter] = '\0';
      if (receivedSerialCMD[0] == 'R')
      { 
        TestButton = true; 
      }
    }
  }
}