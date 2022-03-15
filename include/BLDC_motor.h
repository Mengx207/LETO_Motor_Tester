#include <stdint.h>
#ifndef BLDC_motor_H
#define BLDC_motor_H

#define Motor_Speedbuff_size 9
#define Motor_FindPhotoGateSpeed 3400
#define defauleMotorPWM 3660 // 3540
#define defauleMotorPWM_V 3406
#define motorReturnSpeed 3200

#define stopMotorPWM 4095       // Motor control duty cycle for stop motor rotation
#define targetMirrorSpeed 17930 // microsecond
#define speedErrorThreshold 35

#define MAX_PWMCtrl_Adjust 25
#define MAX_PWMOutput 3700
#define MIN_PWMOutput 3000

#define ScanRange_H 200
#define ScanRange_V 300

// 3600 6V 22.37ms
// 3550 6V 18.69ms
// 3540 6V 17.91ms

#define StopBand 3

#define CW true
#define CCW false

#define M2_dir_pin 37
#define M2_enCoder_pin 2
#define M2_pwmCtl_pin 36
#define M2_absEncoderCS_pin 9

#define TriggerFromCamear_pin 16
#define TriggerFromCamear_pin2 17

#define LC_245IN_En 20
#define LC_245Out_En 21

#define camera_trigger_pin 15
#define shutterLSW_Single 29
#define shutterLSW_Double 30
#define shutterOverCurrentFaltPin 28
#define shutterCurrentFbPin_Single 25
#define shutterCurrentFbPin_Double 24
#define shutterMotorEn_Pin 22

#define photoGatePinH 5

#define ScanIncCount 45

enum motorInitStatus
{
  notInit = 0,
  horiziontalInited,
  veriticalInited,
  Initilized,
};

enum stateMachineStatus
{
  InitialState = 0,
  IdleState,
  CalibrationState,
  HorizontalScanState,
  VerticalScanState,
  Return2ScanState,
  ReportState,
  ErrorHandleState,
};

enum systemErrorCode
{
  noError = 0,
  photoGateDetectionTimeoutError,
  shutterOverCurrentError,
  gearedMotorTimeoutError,
  motorControlOverRideError,
  horiziontalMotorScanError,
  verticalMotorScanError,
  readAbsoluteEncoderError,
  gambalNotCalibriatedError,
  gambalCalibrationDriftError,
  findingPhotoGateError,
};

struct stateMachineType
{
  stateMachineStatus State = InitialState;
  volatile bool requireInitilization = true;
  volatile bool ready4NextState = false;
  stateMachineStatus nextState;
  stateMachineStatus previousState;
  systemErrorCode stateError = noError;
};

struct EEPROM_dataType
{
  /* data */
  char gimbalName[20];
  uint16_t absoluteEncoderZero_H;
  uint16_t absoluteEncoderZero_V;
  uint16_t absStartPos_H;
  uint16_t absStartPos_V;
  uint16_t absRange_H;
  uint16_t absRange_V;
  uint16_t incStartPos_H;
  uint16_t incStartPos_V;
  uint8_t factoryInitilized = 0;

  uint16_t ScanStartPos_H = 6000;
  uint16_t ScanStartPos_V = 300;

  double P_Coef_V;
  double D_Coef_V;
  double I_Coef_V;
  double P_Coef_H;
  double D_Coef_H;
  double I_Coef_H;
};

struct BLDC_motor
{
  volatile bool rotationDirection;
  volatile bool checkHallSensor;
  volatile bool motorIncRunning;
  volatile bool motorAbeRunning;
  volatile bool motorEncabled = false;
  char motorType = 'H';
  uint8_t directionPin;
  uint8_t PWM_contolPin;
  uint8_t encoderOutputPin; // build-in increatimal encoder pin
  uint8_t sensorPin;        // Hall/photogate sensor input pin. Low when blocked
  uint8_t absoluteEncoder_CS_pin;
  uint16_t motorControlDuty = defauleMotorPWM;
  uint32_t timeOut;
  uint16_t absoluteEncoder_trigger_pos;
  uint16_t abeTarget;
  int16_t incTarget;
  int16_t currIncPosition;
  uint16_t currAbePosition;
  volatile uint16_t ABE_raw;
  bool motorInPosition = false;
  int ErrorCode = 0;
  //AS5048A *angleSensor;
};

bool setIncTargetPosition(BLDC_motor *_motor, int16_t _nextPos,
                          uint16_t _speed = defauleMotorPWM);
void go2Inc_run(BLDC_motor *_motor);
bool go2ABEPosition(BLDC_motor *_motor, uint16_t _nextPos, uint16_t _speed = defauleMotorPWM_V);
void go2ABE_run(BLDC_motor *_motor);
void stopMotor(BLDC_motor *_motor);
bool readABE_raw(BLDC_motor *_motor);
void initMotor(BLDC_motor *_motor, uint8_t _ctlPin = M2_pwmCtl_pin,
               uint8_t _dirPin = M2_dir_pin,
               uint8_t _SPIcsPin = M2_absEncoderCS_pin,
               uint8_t _incEncPin = M2_enCoder_pin);
void resetCurrentPosition(BLDC_motor *_motor);
void findPhotogate(BLDC_motor *_motor);
void checkPhotogate(BLDC_motor *_motor);
void motorRun(BLDC_motor *_motor);
void setMotorCalibrationData(BLDC_motor *_motor);

#endif