#include "LetoBldc.h"
#include "writeCalibtationData.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);

#define targetSpeed 158
LETO_BLDC_Motor Motor;
uint8_t serialBitCounter = 0;
u_int16_t TargetLocation_1= 3000;
u_int16_t TargetLocation_2= 5000;
bool TestButton = false;
bool NewMotor = true;
bool WriteData = false;
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
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  display.display();
  Serial.println("IO test");
  // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Welcome tester!");
  display.setCursor(0,16);
  display.println("Motor type:");
  display.println("LETO BLDC");
  display.println("OVU00243EndStop");
  display.println("OVU00244Continuous");
  display.println("---------------------");
  display.println("Test starts soon.....");
  display.startscrollright(7,7);
  display.display(); // actually display all of the above
  delay(2000);
}
void loop()
{
  if(NewMotor == true)
  {
    Motor.I2C_addr = 0x29;
    Motor.begin();
    MotorInitial();
    Serial.printf("Moving to target location: %d\r\n", TargetLocation_1);
    display.printf("Moving to target location: %d\r\n", TargetLocation_1);
    Motor.gotoAbsoluteLocationAtSpeed(TargetLocation_1, 2 * targetSpeed);
    delay(200);
    while (Motor.isMotorMoving() != 0)
    {
      delay(1000);
    }
    Serial.printf("Encoder Reading: v: %d\r\n", Motor.getEncoderReading());
    Serial.println("Ready for test, send \"$R\" command to start test");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Ready to test:");
    display.println("Press R to run test");
    display.display();

    NewMotor = false;
  }

  processSerialCMD();
  if(TestButton == true)
  {
    display.stopscroll();
    Serial.println("Motor check and test:");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Motor check and test:");
    display.display();
    if(CheckPID()&&PowerOnSleep()&&IsMotorMoving()&&MechRange()&&CheckTemp()&&CheckProtection())
    {
      Serial.println("Motor is ready to assemble.");
      display.setCursor(0,8);
      display.println("**Ready to Assemble**");
      display.display();
    }
    else
    {
      Serial.println("Something is wrong with Motor.");
      display.println("|Error in Motor|");
      display.display();
    }
    TestButton = false;
  }
  if(WriteData == true)
  {
    char type[0];
    if(Motor.I2C_addr == 0x29)
    {
      Serial.println("Motor type: V");
      Motor.writeRecommendPID_Data(1);
    }
    if(Motor.I2C_addr == 0x28)
    {
      Serial.println("Motor type: H");
      Motor.writeRecommendPID_Data(0);
    }
    Motor.saveSettingsToFlash();
    Serial.printf("Write Motor PID: %d, %d, %d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
    delay(3000);
    Motor.stopMotor();
    WriteData = false;
  }
}

bool CheckPID()
{
    if(Motor.get_P_Gain() == 1019 && Motor.get_I_Gain() == 5 && Motor.get_D_Gain() == 1024)
    {
        Serial.println("Motor PID is checked");
        display.setCursor(0,16);
        display.println("PID:check");
        display.display();
        return true;
    }
    else
    {
        Serial.printf("Motor PID error: %d, %d, %d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
        display.setCursor(0,16);
        display.println("PID:error");
        display.display();
        return false;
    }
}
bool CheckEndStop()
{
    if(Motor.getFirstEndstop() == 100)
    {
        Serial.println("Endstop is checked");
        display.println("Endstop:check");
        display.display();
        return true;
    }
    else
    {
        Serial.printf("Endstop error: %d\r\n", Motor.getFirstEndstop());
        display.println("Endstop:error");
        display.display();
        return false;
    }
}
bool CheckTemp()
{
    if(Motor.getTemperature()<50)
    {
        Serial.println("Temperature is checked");
        display.printf("Temp:check(%dC)\r\n", Motor.getTemperature());
        display.display();
        return true;
    }
    else
    {
        Serial.printf("Over temperature: %d\r\n", Motor.getTemperature());
        display.printf("Temp:over(%dC)\r\n", Motor.getTemperature());
        display.display();
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
    display.println("Temp protection:check");
    display.display();
    return true;
  }
}
bool IsMotorMoving()
{   
    Motor.gotoAbsoluteLocationAtSpeed(TargetLocation_2, 2 * targetSpeed);
    if(TargetLocation_2 >= 5000)
    {
      TargetLocation_2 = 1000;
    }
    else
    {
      TargetLocation_2 = TargetLocation_2 * 2;
    }
    if(Motor.isMotorMoving() == 0)
    {
        Serial.println("Motor is not moving.");
        display.println("Movement:error");
        display.display();
        return false;
    }
    else
    {
        Serial.println("Motor's move is checked");
        display.println("Movement:check");
        display.display();
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
    display.println("Sleepmode:check");
    display.display();
    return true;
  }
}
bool MechRange()
{
  if(Motor.getMechanicalRange() == 5100)
  {
    Serial.println("Mechanical range is checked");
    display.println("Mech range:check");
    display.display();
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
  display.stopscroll();
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Initializing motor:");
  display.setCursor(0,8);
  display.println("(V motor)");
  display.display();
  delay(1000);

  Motor.set_P_Gain(1019);
  Motor.set_I_Gain(5);
  Motor.set_D_Gain(1024);
  if(Motor.get_P_Gain() && Motor.get_D_Gain())
  {
    display.setCursor(0,16);
    Serial.println("I2C is ON");
    display.println("I2C is ON");
    display.display();
  }
  else
  {
    Serial.println("I2C is OFF");
    display.setCursor(0,16);
    display.println("I2C is OFF");
    display.display();
  }
  Serial.printf("Motor PID: %d, %d, %d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
  display.setCursor(0,24);
  display.printf("P/I/D:%d/%d/%d\r\n", Motor.get_P_Gain(),Motor.get_I_Gain(),Motor.get_D_Gain());
  display.display();
  
  Motor.setFirstEndstop(100);
  Serial.printf("Motor endstop: %d\r\n",Motor.getFirstEndstop());
  display.setCursor(0,32);
  display.printf("Endstop: %d\r\n",Motor.getFirstEndstop());
  display.display();
  Motor.setMechanicalRange(5100);
  Motor.getMechanicalRange();
  display.setCursor(0,40);
  display.println("Temp protection:on");
  display.println("---------------------");
  display.println("Wait for homing......");
  display.startscrollright(7,7);
  display.display();
  Motor.setTempProtection(true);
  Motor.setSleepOnPowerUpMode(false);
  Motor.saveSettingsToFlash();
  delay(1000);


  bool waitHoming = false;
  u_int32_t timer = millis();
  while (!waitHoming)
  {
    waitHoming = Motor.finishedHoming();
    delay(100);
    if(millis()-timer > 30000)
    {
      Serial.println("Homing time out");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Homing time out");
      display.display();

    }
  }
  delay(3000);
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
      if(receivedSerialCMD[0] == 'N')
      {
        NewMotor = true;
      }
      if(receivedSerialCMD[0] == 'W')
      {
        WriteData = true;
      }
    }
  }
}