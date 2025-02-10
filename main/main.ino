#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESC.h>

MPU6050 mpu;

//PINI COMUNICARE
//SDA = A4
//SCL = A5

//parametrii IMU
int deadzone = 8;
float alfa = 0.5;
float angleX;
int16_t accY, accZ;
float accAngle;
int16_t gyroX, gyroRate;
float gyroAngle = 0;

//esantionare
float currTime, prevTime = 0, loopTime;

//pinout
int esc1_pin = 10;
int esc2_pin = 9;

ESC ESC1(esc1_pin, 1000, 2000, 1000);  //M1
ESC ESC2(esc2_pin, 1000, 2000, 1000);  //M2

//PID
//erorile in timp
float ups1 = 0;
float upt1 = 0;
float yps1 = 0;
float ypt1 = 0;
//parametrii ecutiatiei cu diferente
float upp1[3] = { 0.037556772508577, 0.030728268395416, 0 };
float ypp1[3] = { 1, -1, 0 };

float ups2 = 0;
float upt2 = 0;
float yps2 = 0;
float ypt2 = 0;

float upp2[3] = { -13.100701652605988, 13.043720316359739, 0 };
float ypp2[3] = { 1, -1, 0 };

int res1, res2, speed1, speed2;

//filtru butterworth
const double f_s = 1 / 0.01;
const double f_c = 1;
const double f_n = 2 * f_c / f_s;
auto filter = butter<2>(f_n);

//filtru la filtru
float smooth(float newVal) {
  static float oldVal = 0;
  float sum;
  sum = (oldVal * 7 + newVal) / 8;
  oldVal = sum;
  return oldVal;
}

float controller(float pos, float target_pos, int esc_pin,
                 float &ups, float &upt, float &yps, float &ypt, float upp[], float ypp[]) {
  //filtrare
  float pos_actual = filter(smooth(pos));  //unghiul actual al ansamblului

  //PID
  float up = target_pos - pos_actual;
  if (up < 3 && up > -3) up = 0;
  float utp = upp[0] * up + upp[1] * ups + upp[2] * upt;
  float ytp = -yps * ypp[1] - ypt * ypp[2];

  //for not stacking too much
  if (ytp < -100) ytp = -100;
  if (ytp > 100) ytp = 100;
  float yp = utp + ytp;
  // and errors
  upt = ups;
  ups = up;
  ypt = yps;
  yps = yp;

  //mapam valorea yp-ului in intervalul [1100, 1150]
  //yp = map(yp, 1000, 1250, 1100, 1150);

  // Serial.print(up); //eroarea
  // Serial.print(", ");
  // Serial.print(utp);
  // Serial.print(", ");
  // Serial.print(ytp);
  // Serial.print(", ");
  // Serial.print(int(yp)); // viteza actuala
  // Serial.print(", ");
  // Serial.print(pos_actual); //unghiul la care se afla
  // Serial.println();

  return yp;
}

void setup() {
  Serial.begin(115200);

  //activam on bord lowpass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(2);
  Wire.endTransmission(true);

  mpu.initialize();

  //offset urile
  mpu.setXAccelOffset(-3548);
  mpu.setYAccelOffset(-662);
  mpu.setZAccelOffset(1304);
  mpu.setXGyroOffset(-24);
  mpu.setYGyroOffset(152);
  mpu.setZGyroOffset(7);

  ESC1.arm();
  ESC2.arm();

  delay(3000);
}

void loop() {
  //calculam Te
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  //vodoo magic
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  if (gyroRate < deadzone || gyroRate > -deadzone) gyroRate = 0;
  gyroAngle = gyroAngle + (float)gyroRate * loopTime / 1000;
  angleX = alfa * accAngle + (1 - alfa) * gyroAngle;
  angleX *= 2;
  angleX = smooth(angleX);

  res1 = controller(angleX, 0, esc1_pin, ups1, upt1, yps1, ypt1, upp1, ypp1);
  res2 = controller(angleX, 0, esc2_pin, ups2, upt2, yps2, ypt2, upp2, ypp2);

  Serial.print(res1);
  Serial.print(",");
  Serial.print(res2);
  Serial.print(";");

  speed1 = map(res1, -125, 125, 1080, 1123);
  speed2 = map(res2, -125, 125, 1030, 1130);

  ESC1.speed(speed1);
  ESC2.speed(speed2);

  Serial.print(speed1);
  Serial.print(",");
  Serial.print(speed2);
  Serial.print(",");
  Serial.println(angleX);

  // Serial.println(loopTime);

  delay(8);
}