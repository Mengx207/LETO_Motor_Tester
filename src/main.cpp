#include "LetoBldc.h"
#include "writeCalibtationData.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>

LETO_BLDC_Motor vAxialMotor;
LETO_BLDC_Motor hAxialMotor;

char receivedSerialCMD[10];
uint8_t serialBitCounter = 0;
bool TestButton = false;

void CheckPID();
void CheckEndStop();
void CheckTemp();
void CheckProtection();
void MotorInitial();
void processSerialCMD();

void setup()
{
  Serial.begin(115200);
  hAxialMotor.I2C_addr = 0x50 >> 1;
  vAxialMotor.I2C_addr = 0x52 >> 1;
  hAxialMotor.name[0] = 'H';
  vAxialMotor.name[0] = 'V';
  hAxialMotor.begin();
  MotorInitial();
}

void loop()
{
  processSerialCMD();
    if (TestButton == true)
    {
        Serial.println("Motor check and test:");
        CheckPID();
        CheckEndStop();
        CheckTemp();
        CheckProtection();
        TestButton = false;
    }
}
void CheckPID()
{
    //Check PID
    if(vAxialMotor.get_P_Gain() == 2400)
    {
        Serial.println("vP is checked");
    }
    else
    {
        Serial.println("vP error");
    }
    if(vAxialMotor.get_I_Gain() == 0)
    {
        Serial.println("vI is checked");
    }
    else
    {
        Serial.println("vI error");
    }
    if(vAxialMotor.get_D_Gain() == 2000)
    {
        Serial.println("vD is checked");
    }
    else
    {
        Serial.println("vD error");
    }

    if(hAxialMotor.get_P_Gain() == 1200)
    {
        Serial.println("hP is checked");
    }
    else
    {
        Serial.println("hP error");
    }
    if(hAxialMotor.get_I_Gain() == 0)
    {
        Serial.println("hI is checked");
    }
    else
    {
        Serial.println("vI error");
    }
    if(hAxialMotor.get_D_Gain() == 1900)
    {
        Serial.println("vD is checked");
    }
    else
    {
        Serial.println("vD error");
    }
}
void CheckEndStop()
{
    //Check End Stop
    if(vAxialMotor.getFirstEndstop() == 100)
    {
        Serial.println("vEndstop is checked");
    }
    else
    {
        Serial.println("vEndstop error");
    }
    if(hAxialMotor.getFirstEndstop() == 100)
    {
        Serial.println("hEndstop is checked");
    }
    else
    {
        Serial.println("hEndstop error");
    }
}
void CheckTemp()
{
    if(vAxialMotor.getTemperature()<50)
    {
        Serial.println("vTemperature is checked");
    }
    else
    {
        Serial.println("vOver temperature.");
    }

    if(hAxialMotor.getTemperature()<50)
    {
        Serial.println("hTemperature is checked");
    }
    else
    {
        Serial.println("hOver temperature.");
    }    
}
void CheckProtection()
{
  vAxialMotor.getTempProtection();
  hAxialMotor.getTempProtection();  
}
void MotorInitial()
{
  delay(5000);
  Serial.println("Initializing motors");
  vAxialMotor.set_P_Gain(2400);
  vAxialMotor.set_I_Gain(0);
  vAxialMotor.set_D_Gain(2000);
  Serial.printf("V motor PID: %d, %d, %d\r\n", vAxialMotor.get_P_Gain(),vAxialMotor.get_I_Gain(),vAxialMotor.get_D_Gain());
  
  vAxialMotor.setFirstEndstop(100);
  vAxialMotor.getFirstEndstop();
  vAxialMotor.setMechanicalRange(5100);
  vAxialMotor.getMechanicalRange();
  vAxialMotor.setSleepOnPowerUpMode(false);
  vAxialMotor.setTempProtection(true);
  //vAxialMotor.saveSettingsToFlash();
  //vAxialMotor.resetMotor();
  delay(2000);

  hAxialMotor.set_P_Gain(1200);
  hAxialMotor.set_I_Gain(0);
  hAxialMotor.set_D_Gain(1900);
  Serial.printf("H motor PID: %d, %d, %d\r\n", hAxialMotor.get_P_Gain(),hAxialMotor.get_I_Gain(),hAxialMotor.get_D_Gain());
  hAxialMotor.setFirstEndstop(100);
  hAxialMotor.getFirstEndstop();
  hAxialMotor.setMechanicalRange(7500);
  hAxialMotor.getMechanicalRange();
  hAxialMotor.setSleepOnPowerUpMode(false);
  hAxialMotor.setTempProtection(true);
  //hAxialMotor.saveSettingsToFlash();
  //hAxialMotor.resetMotor();
  delay(2000);
  Serial.println("Initialization finished");

}
void processSerialCMD()
{
  while (Serial.available())
  {
    char tempRead = Serial.read();
    // Serial.print(tempRead);
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
      // Serial.println(receivedSerialCMD);
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