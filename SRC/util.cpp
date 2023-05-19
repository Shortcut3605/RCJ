#ifndef UTIL_CPP
#define UTIL_CPP

#include "global.h"
int tcaselect(uint8_t i)
{
  if (i > 7) return -4;
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  return (Wire.endTransmission());
}

void setSpeed(double left, double right) {
  RightMotor.setMotorPwm((right * -255));
  LeftMotor.setMotorPwm((left * 255));
}

void setSpeed(double val, bool left) {
    if (left == 0) {
      RightMotor.setMotorPwm((val * -255));
    }
    else {
      LeftMotor.setMotorPwm((val * 255));
    }
}

void isr_LeftMotorEncoder(void)
{
if (digitalRead(LeftMotor.getPortB()) == 0) {
LeftMotor.pulsePosMinus();
} else {
LeftMotor.pulsePosPlus();
}
}

void isr_RightMotorEncoder(void)
{
if (digitalRead(RightMotor.getPortB()) == 0) {
RightMotor.pulsePosMinus();
} else {
RightMotor.pulsePosPlus();
}
}

void initPWM8kHz() 
{
TCCR1A = _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(WGM12);
TCCR2A = _BV(WGM21) | _BV(WGM20);
TCCR2B = _BV(CS21);
}

void attachInterrupt() {
  attachInterrupt(LeftMotor.getIntNum(), isr_LeftMotorEncoder, RISING);
  attachInterrupt(RightMotor.getIntNum(), isr_RightMotorEncoder, RISING);
}

void resetMotors() {
  setSpeed(0.0, 0.0);
  RightMotor.setPulsePos(0);
  LeftMotor.setPulsePos(0);
  RightMotor.updateCurPos();
  LeftMotor.updateCurPos();
}

void move(double cm, double speed, uint8_t type) {
  if(type == MOVEMENT::CM) {
    resetMotors();
    if (cm > 0) {
          setSpeed(speed, speed);
    }
    else {
      setSpeed(-speed, -speed);
    }
    double enc = abs((cm/(WR * PI)) * 360);
    do {
      RightMotor.updateCurPos();
      LeftMotor.updateCurPos();
      } while (abs(RightMotor.getCurPos()) < (enc) && abs(LeftMotor.getCurPos()) < enc);
        resetMotors();
  }
  
  else {
    resetMotors();
    if (cm > 0) {
      setSpeed(speed, -speed);
    }
    else {
      setSpeed(-speed, speed);
    }
    double enc = abs((WB * (cm/WR)));
    do  {
      RightMotor.updateCurPos();
      LeftMotor.updateCurPos();
      } while((abs(RightMotor.getCurPos()) < (enc) && abs(LeftMotor.getCurPos()) < enc));
    resetMotors();
  }
}

void followLine() {
  uint8_t line_value = lineFollower.readSensors();
  int speed = 0.4;
  int ang = 2;
  int cm = 1;
  switch (line_value) {
    case LB_RB:  // both sensors see black
      move(0.5, 0.3, MOVEMENT::CM);
      break;
    case LW_RB:  // S1 sees white , S2 sees black
      move(3, 0.3, MOVEMENT::ANG);
      break;
    case LB_RW:  // S1 sees black , S2 sees white
      move(-3, 0.3, MOVEMENT::ANG);
      break;
    case LW_RW:  // both sensors see white
      move(0.5, 0.3, MOVEMENT::CM);
      break;
  }
}


void gyroTurn(double ang, double speed) {
    gyro.reset();
    gyro.update();
    if (ang < 0) { 
      setSpeed(-speed, speed); 
      while (gyro.getAngleZ() > ang && gyro.getAngleZ() < 0 || gyro.getAngleZ() > -5 && gyro.getAngleZ() < 5) { gyro.update(); }
      }
    else { 
      setSpeed(speed, -speed); 
      while (gyro.getAngleZ() < ang && gyro.getAngleZ() > 0 || gyro.getAngleZ() < -5 && gyro.getAngleZ() > 5) { gyro.update(); }
    }
    setSpeed(0.0, 0.0);
}


void irAvoid(double r1, double speed) {
    double q, r2 = 7.3;
    bool left;
    q = r2 - (WBL/2) + (WBW/2);
    setSpeed(0.2, 0.2);
    while (usSensor.distanceCm() > q);
    setSpeed(0.0, 0.0);
    delay(100);
    gyroTurn(-90, 0.5);
    if (usSensor.distanceCm() > 20) {left = true;}
    else {
      left = false;
      gyroTurn(180, 0.5);
    }
    double lspeed, rspeed;
    lspeed = speed;
    rspeed = ((r1 + r2)/(r1 + r2 + WBL) * speed);
    move(3, 0.3, MOVEMENT::CM);
    if (left == false) {setSpeed(rspeed, lspeed);}
    else {setSpeed(lspeed, rspeed);}
    color.takeMeasurements();
    while(!(color.getCalibratedGreen() < 50)) {
      color.takeMeasurements();
    }
    setSpeed(0.0, 0.0);
    delay(1000);
    move(5, 0.4, MOVEMENT::CM);
    move(70, 0.5, MOVEMENT::ANG);
    delay(1000);
}

void QTRCalibrate() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);
  Serial.print("Calibrating... Wait 10 seconds.");
  for (uint16_t i = 0; i < 400; i++) // 200 should be 400
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
    Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

/*void LineFollow() {
   uint16_t position = qtr.readLineBlack(sensorValues);
   long long int bitmask = 0;
   //for(int i = 0; i < 8; i++){if(sensorValues[i]){}}
  //  Serial.print(position);
   double pos_double = position;
   double error = ((pos_double - 3500.0) / 3500.0); // 0.7 is our KP
   error *= 2.0f;
  //  Serial.print(" Err: ");
  //  Serial.print(error);
   double errorDiff = error - p_error;
   errorDiff *= 0.01;
   //Serial.print("Derivative: ");
   //Serial.println(errorDiff);
   bool allWhite = true;
   for(int i = 0; i < 8; i++)
   {
      Serial.print(sensorValues[i]);
     Serial.print(" ");
     if(sensorValues[i] > 120) {allWhite = false; break;}
   }
   Serial.println();
   double baseValue = 0.3;
   if(errorDiff >= 1 && !allWhite){baseValue = 0.05;}
   if(allWhite){error = 0; errorDiff = 0; Serial.println("All White");}
   double lMotorValue = baseValue - error - (errorDiff); // 0.3 is our initial speed. intial speed + KP = 1
   double rMotorValue = baseValue + error + (errorDiff);

   lMotorValue = constrain(lMotorValue, -0.8f, 0.8f);
   rMotorValue = constrain(rMotorValue, -0.8f, 0.8f);
   p_error = error;
   unsigned long start = millis();
   setSpeed(rMotorValue, lMotorValue);
   unsigned long elapsed = millis() - start;
  // Serial.print("Set speed took: ");
  // Serial.println(elapsed);
}*/

void GreenCalibrate() {
  
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial2.print("ATTCSMD\n");
  Serial3.print("ATTCSMD\n");
  delay(3000);
    Serial3.print("ATLED1=100\n");
  Serial2.print("ATLED1=100\n");
  delay(1000);
  Serial.println(Serial2.available());
  while(Serial2.available())
     Serial.print((char)Serial2.read());
  Serial2.print("ATINTTIME=1\n");
  delay(1000);
  Serial2.print("ATGAIN=3\n");
  delay(1000);
  Serial.println(Serial3.available());
  while(Serial3.available())
     Serial.print((char)Serial3.read());
  Serial3.print("ATINTTIME=1\n"); 
  delay(1000);
  Serial3.print("ATGAIN=3\n");
  delay(1000);

  Serial.println(Serial3.available());
  while(Serial3.available())
     Serial.print((char)Serial3.read());
  Serial3.print("ATINTTIME\n");
  Serial.println("int time");
  delay(1000);
   Serial.println(Serial2.available());
  while(Serial2.available())
     Serial.print((char)Serial2.read());
  Serial2.print("ATINTTIME\n");
  Serial.println("int time");
  delay(1000);

  Serial.println(Serial3.available());
  while(Serial3.available())
     Serial.print((char)Serial3.read());
  Serial.println(Serial2.available());
  while(Serial2.available())
     Serial.print((char)Serial2.read());

  delay(5000);
  Serial.println("Ready!");
}

void HasGreen() {
  
}
#endif
