#include "Arduino.h"
#include "EEPROM.h"
#include "writeCalibtationData.h"
extern EEPROM_dataType InitiEEPROM_data;

void writeData(int _addr, EEPROM_dataType *_calibrationData) {
  int dataSize = sizeof(EEPROM_dataType) / sizeof(uint8_t);
  int tempAddr = _addr * dataSize;
  uint8_t *tempData = (uint8_t *)_calibrationData;
  for (int i = 0; i < dataSize; i++) {
    EEPROM.write(tempAddr + i, *tempData++);
  }
}

void readData(int _addr, EEPROM_dataType *_calibrationData) {
  int dataSize = sizeof(EEPROM_dataType);
  int tempAddr = _addr * dataSize;
  uint8_t tempData[dataSize];
  for (int i = 0; i < dataSize; i++) {
    tempData[i] = EEPROM.read(tempAddr++);
  }
  memcpy(_calibrationData, tempData, dataSize);
  memcpy(&InitiEEPROM_data, tempData, dataSize);
  // printCalibrationData(&InitiEEPROM_data);
  //   _calibrationData = (EEPROM_dataType *)tempData;
}

void printCalibrationData(EEPROM_dataType *_caibrtaitonData) {
  Serial.println("Calibration Data Print");
  Serial.print("absoluteEncoderZero_V: ");
  Serial.println(_caibrtaitonData->absoluteEncoderZero_V);
  Serial.print("absoluteEncoderZero_H: ");
  Serial.println(_caibrtaitonData->absoluteEncoderZero_H);
  Serial.print("absRange_H: ");
  Serial.println(_caibrtaitonData->absRange_H);
  Serial.print("absStartPos_H: ");
  Serial.println(_caibrtaitonData->absStartPos_H);
  Serial.print("absStartPos_V: ");
  Serial.println(_caibrtaitonData->absStartPos_V);
  Serial.print("D_Coef_H: ");
  Serial.println(_caibrtaitonData->D_Coef_H);
  Serial.print("D_Coef_V: ");
  Serial.println(_caibrtaitonData->D_Coef_V);
  Serial.print("factoryInitilized: ");
  Serial.println(_caibrtaitonData->factoryInitilized);
  Serial.print("gimbalName: ");
  Serial.println(_caibrtaitonData->gimbalName);
  Serial.print("incStartPos_H: ");
  Serial.println(_caibrtaitonData->incStartPos_H);
  Serial.print("incStartPos_V: ");
  Serial.println(_caibrtaitonData->incStartPos_V);
  Serial.print("ScanStartPos_H: ");
  Serial.println(_caibrtaitonData->ScanStartPos_H);
  Serial.print("ScanStartPos_V: ");
  Serial.println(_caibrtaitonData->ScanStartPos_V);
  Serial.println("End");
}
