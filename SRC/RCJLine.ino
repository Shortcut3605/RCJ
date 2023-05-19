#include "global.h"
#include "util.h"
#include "LineFollower.h"
#include "QTRSensors.h"
MePort_Sig mePort[17] = { { NC, NC }, { 5, 4 }, { 3, 2 }, { 7, 6 }, { 9, 8 }, { 16, 17 }, { A10, A15 }, { A9, A14 }, { A8, A13 }, { A7, A12 }, { A6, A11 }, { NC, A2 }, { NC, A3 }, { NC, A0 }, { NC, A1 }, { NC, NC }, { NC, NC } };
Encoder_port_type encoder_Port[6] = { { NC, NC, NC, NC, NC }, { 19, 42, 11, 49, 48 }, { 18, 43, 10, 47, 46 }, { NC, NC, NC, NC, NC }, { NC, NC, NC, NC, NC } };
MeEncoderOnBoard RightMotor(SLOT1);
MeEncoderOnBoard LeftMotor(SLOT2);
MeLineFollower lineFollower(LINE_PIN);
MeGyro gyro(0, gyroPin);
MeUltrasonicSensor usSensor(ULTRASONIC_PIN);
AS726X color;
uint16_t sensorValues[SensorCount];
QTRSensors qtr;
double p_error = 0;

void setup() {
  Serial.begin(9600);
  QTRCalibrate();
  GreenCalibrate();
  attachInterrupt();
}

int i = 0;

bool lGreen() {
  if (Serial2.available()) {
    Serial2.read();
    int v = Serial2.parseInt();
    int b = Serial2.parseInt();
    int g = Serial2.parseInt();
    int y = Serial2.parseInt();
    int o = Serial2.parseInt();
    int r = Serial2.parseInt();
    double avg = (v + b + g + y + o + r) / 6.0;
    double newG = g * 1.0f;
    double newB = b * 1.0f;
    double newO = o * 1.0f;
    double newY = y * 1.0f;
    double newR = r * 1.0f;
    double rp = newG / newY;
    double rp2 = newG / newO;
    double rp3 = newR / newG;
    //sprintf(buff, "%d %d %d %d %d %d %d", v, b, g, y, o, r, (int)avg);

    // Serial.print(buff);
    //  Serial.print(" ");
    //  Serial.print(" ");
    //+   if (rp >= 1.25f && rp <= 1.32f && rp2 >= 1.7f) {Serial.print("GREEN");}
    //  Serial.println();
    /* Serial.print("RP: ");
    Serial.print(rp);
    Serial.print(" RP2: ");
    Serial.println(rp2);*/
    Serial.println(rp2);
    if (rp2 > 1.8 && rp3 < 0.1) {  // right -> rp >= 1.20 && rp2 >= 1.60 && rp2 <= 2.5
      Serial.print("GREEN");
      resetMotors();
      // delay(1000); -> removed
      // move(FORWARD_CHECK, 0.4, MOVEMENT::CM);
      // CHECKING FOR GREEN
      /* delay(1000);  //
           int position = qtr.readLineBlack(sensorValues);
           int sum = 0;
           bool wrk = true;
           for (int j = 0; j < 8; j++) {
             sum += sensorValues[i];
             Serial.print(sensorValues[i]);
             Serial.print(" ");
             if(sensorValues[i] < 90)
             {
               wrk = false;
             }
           }
           Serial.println();
           double avg2 = (double)sum;
           delay(1000); //added*/
      if (true) {  // avg2 > THRESHOLD_AVG2 || avg2 < 50

        return true;
        //  move(-FORWARD_CHECK-0.1f, 0.4, MOVEMENT::CM);
        delay(1000);
      }
    }
  }
  return false;
}

bool rGreen() {
  if (Serial3.available()) {
    Serial3.read();
    int v = (int)Serial3.parseInt();
    int b = (int)Serial3.parseInt();
    int g = (int)Serial3.parseInt();
    int y = (int)Serial3.parseInt();
    int o = (int)Serial3.parseInt();
    int r = (int)Serial3.parseInt();
    double avg = (v + b + g + y + o + r) / 6.0;
    /*Serial.print(v);
    Serial.print(" ");
    Serial.print(g);
    Serial.print(" ");
    Serial.print(b);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(o);
    Serial.print(" ");
    Serial.print(r);
    Serial.println(" ");*/
    double newG = g * 1.0f;
    double newB = b * 1.0f;
    double newO = o * 1.0f;
    double newY = y * 1.0f;
    double newR = r * 1.0f;
    double rp = newG / newY;
    double rp2 = newG / newO;
    double rp3 = newY / newO;
    double rp4 = (newR) / newG;
    /* Serial.print(rp4);
  Serial.print(" ");
  Serial.println(rp2);*/
    //Serial.println(rp3);
    // Serial.println(rp4);
    if ((rp4 < 0.1 && rp2 > 1.3f && rp2 < 1.6f)) {  //
      Serial.print(" GREEN B1 ");
      Serial.print(rp);
      Serial.print(" ");

      Serial.println(rp2);
      resetMotors();
      delay(1000);
      /* move(FORWARD_CHECK, 0.4, MOVEMENT::CM);
       // CHECKING FOR GREEN
       //delay(1000); //
       int position = qtr.readLineBlack(sensorValues);
       int sum = 0;
       bool wrk = true;
        for (int j = 0; j < 8; j++) {
             sum += sensorValues[i];
             Serial.print(sensorValues[i]);
             Serial.print(" ");
             if(sensorValues[i] < 90)
             {
               wrk = false;
             }
           }
           Serial.println();
       double avg2 = (double)sum;
       delay(1000); // added*/
      if (true) {

        return true;
        //  move(-FORWARD_CHECK, 0.4, MOVEMENT::CM);
        delay(1000);
      }
    }
  }
  return false;
}

void detectGreen(bool b1, bool b2, int depth) {
  if (depth > 5) { return; }

  const int TURN_SPEED = 80;
  const int THRESHOLD_AVG2 = 1000;  // was 50
  Serial3.print("ATDATA\n");
  Serial2.print("ATDATA\n");
  char buff[100] = "";
  char buf[100] = "";
  const float FORWARD_CHECK = 2.0f;
  if (!b1)
    b1 = rGreen();
  if (!b2)
    b2 = lGreen();
  if (!b1 && !b2) { return; }
  move(0.2f, 0.4f, MOVEMENT::CM);
  delay(500);
  /*if(b2)
  {
    move(0.1f,0.4f,MOVEMENT::CM);
    if(!b1){
      Serial.println("Entering RGREEN");
      Serial3.read();
      b1 = rGreen();
    }
    delay(1000);
   // move(-0.1f,0.4f,MOVEMENT::CM);
  }
  if(b1)
  {
    //move(0.1f,0.4f,MOVEMENT::CM);
    if(!b2)
      b2 = rGreen();
    delay(1000);
    // move(-0.1f,0.4f,MOVEMENT::CM);
  }*/

  if (depth == 5) {
    if (b1 && b2) {
      b1 = b2 = false;
      Serial.println("Double Green");
      delay(5000);
      move(-180, 0.4f, MOVEMENT::ANG);
    } else if (b1) {
      move(FORWARD_CHECK, 0.4, MOVEMENT::CM);
      move(3, 0.4, MOVEMENT::CM);
      move(-TURN_SPEED, 0.4, MOVEMENT::ANG);
      b1 = false;
    } else if (b2) {
      move(FORWARD_CHECK, 0.4, MOVEMENT::CM);
      b2 = false;
      move(3, 0.4, MOVEMENT::CM);
      move(TURN_SPEED, 0.4, MOVEMENT::ANG);
    }
    while (Serial3.available()) Serial3.read();
    while (Serial2.available()) Serial2.read();
  } else {
    detectGreen(b1, b2, depth + 1);
  }
}

void loop() {
  // move(-15, 1, MOVEMENT::ANG);
  //delay(1000);
  detectGreen(false, false, 0);
  // detectGreen();
  uint16_t position = qtr.readLineBlack(sensorValues);
  long long int bitmask = 0;
  //for(int i = 0; i < 8; i++){if(sensorValues[i]){}}
  //  Serial.print(position);
  double pos_double = position;
  double error = ((pos_double - 3500.0) / 3500.0);  // 0.7 is our KP
  error *= 2.0f;
  //  Serial.print(" Err: ");
  //  Serial.print(error);
  double errorDiff = error - p_error;
  errorDiff *= 0.1;  // 0.01
                     // errorDiff = 0;
  //Serial.print("Derivative: ");
  //Serial.println(errorDiff);
  bool allWhite = true;
  for (int i = 0; i < 8; i++) {
    //Serial.print(sensorValues[i]);
    //Serial.print(" ");
    if (sensorValues[i] > 105) {
      allWhite = false;
      break;
    }
    e
  }
  //Serial.println();
  double baseValue = 0.3;
  if (errorDiff >= 1 && !allWhite) { baseValue = 0.05; }
  if (allWhite) {
    error = 0;
    errorDiff = 0;
  }
  double lMotorValue = baseValue - error - (errorDiff);  // 0.3 is our initial speed. intial speed + KP = 1
  double rMotorValue = baseValue + error + (errorDiff);
  const float crf = 0.5f;
  lMotorValue = constrain(lMotorValue, -crf, crf);
  rMotorValue = constrain(rMotorValue, -crf, crf);
  p_error = error;
  unsigned long start = millis();
  setSpeed(rMotorValue, lMotorValue);
  unsigned long elapsed = millis() - start;
  /* Serial.print("Left: ");
    Serial.print(lMotorValue);
    Serial.print("\tRight: ");
    Serial.println(rMotorValue);*/
  // Serial.print("Set speed took: ");
  // Serial.println(elapsed);
}

/*
void loop() { // Walter, if you see this, im going to extract all the code for our PID into a separate function (in the LineFollower file, cus its appropriate for it to be there)
   detectGreen();
   uint16_t position = qtr.readLineBlack(sensorValues);
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
    //  Serial.print(sensorValues[i]);
    // Serial.print(" ");
     if(sensorValues[i] > 140) {allWhite = false; break;} // hardcoded 100
   }
 //  Serial.println();
   double baseValue = 0.3;
   if(errorDiff >= 1 && !allWhite){baseValue = 0.05;}
   if(allWhite){error = 0; errorDiff = 0; Serial.println("All White");}
   double lMotorValue = baseValue - error - (errorDiff); // 0.3 is our initial speed. intial speed + KP = 1
   double rMotorValue = baseValue + error + (errorDiff);

   lMotorValue = constrain(lMotorValue, -0.8f, 0.8f);
   rMotorValue = constrain(rMotorValue, -0.8f, 0.8f);
   p_error = error;
   /* Serial.print("Left: ");
    Serial.print(lMotorValue);
    Serial.print("\tRight: ");
    Serial.println(rMotorValue);*/
/* unsigned long start = millis();
   setSpeed(rMotorValue, lMotorValue);
   unsigned long elapsed = millis() - start; 
  // Serial.print("Set speed took: ");
  // Serial.println(elapsed);
} */
