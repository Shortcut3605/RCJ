#ifndef GLOBAL_H
#define GLOBAL_H
#include <Arduino.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <AS726X.h>
#include <QTRSensors.h>
extern MeEncoderOnBoard RightMotor;
extern MeEncoderOnBoard LeftMotor;
extern MeLineFollower lineFollower;
extern MeGyro gyro;
extern MeUltrasonicSensor usSensor;
extern AS726X color;
extern QTRSensors qtr;
const double WR = 4.01;
const double WB = 20.5;
const double WBL = 15.5;
const double WBW = 12.5;
const int BVALUE = 10;
const int WVALUE = 70;
const int LW_RW = S1_OUT_S2_OUT;
const int LB_RW = S1_IN_S2_OUT;
const int LW_RB = S1_OUT_S2_IN;
const int LB_RB = S1_IN_S2_IN;
const int LINE_PIN = 9;
const int gyroPin = 0x69;
const int ULTRASONIC_PIN = 10;
const uint8_t SensorCount = 8;
namespace MOVEMENT {
  enum : uint8_t {CM, ANG};
}
#endif