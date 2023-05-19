#ifndef LINE_CPP
#define LINE_CPP
#include "global.h"
#include "util.h"
double prev_error = 0.0;
double prev_sum = 0.0;
double lineFollow(double speed)
{
  double KP = 0.6;
  double KI = 0.0;
  double KD = 0.00;
  tcaselect(0);
  color.takeMeasurements();
  double li = color.getCalibratedRed() + color.getCalibratedGreen() + color.getCalibratedBlue();
  li /= 3.0;
  tcaselect(7);
  color.takeMeasurements();
  double ri = color.getCalibratedRed() + color.getCalibratedGreen() + color.getCalibratedBlue();
  ri /= 3.0;
 // uint16_t rv = constrain(color.getCalibratedGreen(), BVALUE, WVALUE);
  double error = (ri - li); //((double)lv - (double)rv) / (WVALUE - BVALUE);
  error = constrain(error, 0.9, -0.9);
  Serial.println(error);
  double sum = prev_sum + error;
  double difference = error - prev_error;
  double adjusta = (double)error * KP + (double)sum * KI + (double)difference * KD; // PID adjustment
  setSpeed(constrain(speed + adjusta, -0.5, 0.5), constrain(speed - adjusta, -0.5, 0.5));
  prev_error = error;
  prev_sum += error;
  return error;
}

void TwoEyeLineTracing()
{
  tcaselect(0);
  color.takeMeasurements();
  double li = color.getCalibratedRed() + color.getCalibratedGreen() + color.getCalibratedBlue();
  li /= 3.0;
  tcaselect(7);
  color.takeMeasurements();
  double ri = color.getCalibratedRed() + color.getCalibratedGreen() + color.getCalibratedBlue();
  ri /= 3;
  
  // DETERMINE AND PRINT OUT STATE
 /*Serial.print("Sensor 1 - ");
  Serial.println(li);
  Serial.print("Sensor 2 - ");
  Serial.println(ri);*/
  Serial.println(li/ri * 15);
  
}

#endif