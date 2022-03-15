#include "LetoBldc.h"
#include "writeCalibtationData.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>

#define targetSpeed 158
LETO_BLDC_Motor vAxialMotor;
LETO_BLDC_Motor hAxialMotor;

uint8_t serialBitCounter = 0;
u_int16_t ScanStartPos_H = 6000;
u_int16_t ScanStartPos_V = 3000;
bool TestButton = false;
char receivedSerialCMD[10];

void CheckPID();
void CheckEndStop();
void CheckTemp();
void CheckProtection();
void MotorInitial();
void processSerialCMD();
void IsMotorMoving();

void setup()
{
  Serial.begin(115200);
  hAxialMotor.I2C_addr = 0x50 >> 1;
  vAxialMotor.I2C_addr = 0x52 >> 1;
  hAxialMotor.name[0] = 'H';
  vAxialMotor.name[0] = 'V';
  hAxialMotor.begin();
  MotorInitial();
  Serial.printf("Moving to start position V: %d, H: %d\r\n", ScanStartPos_V, ScanStartPos_H);
  hAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_H, 2 * targetSpeed);
  vAxialMotor.gotoAbsoluteLocationAtSpeed(ScanStartPos_V, 2 * targetSpeed);
  delay(300);
  IsMotorMoving();
  while (vAxialMotor.isMotorMoving() != 0 || hAxialMotor.isMotorMoving() != 0)
  {
    delay(1000);
  }
  Serial.printf("Encoder Reading: v: %d, h: %d\r\n",
                vAxialMotor.getEncoderReading(),
                hAxialMotor.getEncoderReading());
  Serial.println("Ready for test, send \"$R\" command to start test");
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
    if(vAxialMotor.get_P_Gain() == 2400 && vAxialMotor.get_I_Gain() == 0 && vAxialMotor.get_D_Gain() == 2000)
    {
        Serial.println("V motor PID are checked");
    }
    else
    {
        Serial.printf("V motor PID error: %d, %d, %d\r\n", vAxialMotor.get_P_Gain(),vAxialMotor.get_I_Gain(),vAxialMotor.get_D_Gain());
    }

    if(hAxialMotor.get_P_Gain() == 1200 && hAxialMotor.get_I_Gain() == 0 && hAxialMotor.get_D_Gain() == 1900)
    {
        Serial.println("H motor PID are checked");
    }
    else
    {
        Serial.printf("H motor PID error: %d, %d, %d\r\n", hAxialMotor.get_P_Gain(),hAxialMotor.get_I_Gain(),hAxialMotor.get_D_Gain());
    }
}
void CheckEndStop()
{
    //Check End Stop
    if(vAxialMotor.getFirstEndstop() == 100)
    {
        Serial.println("V Endstop is checked");
    }
    else
    {
        Serial.printf("V Endstop error: %d\r\n", vAxialMotor.getFirstEndstop());
    }
    if(hAxialMotor.getFirstEndstop() == 100)
    {
        Serial.println("H Endstop is checked");
    }
    else
    {
        Serial.printf("H Endstop error: %d\r\n", hAxialMotor.getFirstEndstop());
    }
}
void CheckTemp()
{
    if(vAxialMotor.getTemperature()<50)
    {
        Serial.println("V Temperature is checked");
    }
    else
    {
        Serial.printf("V Over temperature: %d\n\r", vAxialMotor.getTemperature());
    }

    if(hAxialMotor.getTemperature()<50)
    {
        Serial.println("H Temperature is checked");
    }
    else
    {
        Serial.printf("H Over temperature: %d\n\r", hAxialMotor.getTemperature());
    }    
}
void CheckProtection()
{
  vAxialMotor.getTempProtection();
  hAxialMotor.getTempProtection();  
}
void IsMotorMoving()
{
    if(hAxialMotor.isMotorMoving() == 0)
    {
        Serial.println("H Motor is not moving.");
    }
    else if(vAxialMotor.isMotorMoving() == 0)
    {
        Serial.println("V Motor is not moving.");
    }
    else
    {
        Serial.println("Motors move properly.");
    }
}
void MotorInitial()
{
  delay(5000);
  Serial.println("Initializing motors:");
  vAxialMotor.set_P_Gain(2400);
  vAxialMotor.set_I_Gain(0);
  vAxialMotor.set_D_Gain(2000);
  Serial.printf("V motor PID: %d, %d, %d\r\n", vAxialMotor.get_P_Gain(),vAxialMotor.get_I_Gain(),vAxialMotor.get_D_Gain());
  
  vAxialMotor.setFirstEndstop(100);
  Serial.printf("V motor endstop: %d\r\n",vAxialMotor.getFirstEndstop());
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
  Serial.printf("H motor endstop: %d\r\n",hAxialMotor.getFirstEndstop());
  hAxialMotor.setMechanicalRange(7500);
  hAxialMotor.getMechanicalRange();
  hAxialMotor.setSleepOnPowerUpMode(false);
  hAxialMotor.setTempProtection(true);
  //hAxialMotor.saveSettingsToFlash();
  //hAxialMotor.resetMotor();
  bool waitHoming = false;
  while (!waitHoming)
  {
    waitHoming = vAxialMotor.finishedHoming() && hAxialMotor.finishedHoming();
    delay(100);
    // Serial.printf("v: %d, h: %d\r\n", vAxialMotor.getEncoderReading(),
    //               hAxialMotor.getEncoderReading());
  }
  delay(2000);
  Serial.println("Initializatio is finished.");

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