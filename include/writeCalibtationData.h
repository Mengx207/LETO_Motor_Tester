#ifndef WriteCalibtation_H
#define WriteCalibtation_H
#include "BLDC_motor.h"

void writeData(int _addr, EEPROM_dataType * _calibrationData);
void readData(int _addr, EEPROM_dataType * _calibrationData);
void printCalibrationData(EEPROM_dataType * _caibrtaitonData);

#endif