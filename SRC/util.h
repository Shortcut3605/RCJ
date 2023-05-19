#ifndef UTIL_H
#define UTIL_H

int tcaselect(uint8_t);
void setSpeed(double, double);
void setSpeed(double, bool);
void isr_LeftMotorEncoder(void);
void isr_RightMotorEncoder(void);
void attachInterrupt(void);
void initPWM8kHz(void);
void move(double, double, uint8_t);
void resetMotors(void);
void gyroTurn(double, double);
void followLine(void);
void irAvoid(double, double);
void QTRCalibrate(void);
void LineFollow(void);
void GreenCalibrate(void);

#endif