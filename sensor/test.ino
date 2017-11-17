 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
//#include "../libraries/Adafruit_LSM9DS1/Adafruit_LSM9DS1.h"
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>   // for tan2 & fabs

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5 // SCL
#define LSM9DS1_MISO 12 // (SDO_M & SDO_AG)
#define LSM9DS1_MOSI A4 // => SPI MOSI (SDA)
#define LSM9DS1_XGCS 6 // chip select accelerometer (CSAG)
#define LSM9DS1_MCS 5 // chip select magnetometer (CSM)
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// vector_t is just the three float vals x, y, z
#define vector_t sensors_vec_t

// TODO correct sensitivity
#define GYROSCOPE_SENSITIVITY 1
// TODO correct dt
//#define dt 0.01
unsigned long dt = 1;
unsigned long last_time = 1;
unsigned long start_time = 1;

// current position:
// angle around the x, y, z axis
float angleX = 0, angleY = 0, angleZ = 0;

//current velocity and distance
float v = 0, d = 0;

void computeDistance(float a, float delta, float degree){
  //degree: abweichung von ax
  float a_corr = a-sin(degree*PI/180)*9.81+0.001;
  Serial.print("a_corr: "); Serial.print(a_corr);   Serial.print(" ");
  v = v + a_corr*delta;
  //Serial.print("a: "); Serial.print(v);   Serial.print(" ");
  Serial.print("Vel: "); Serial.print(v);   Serial.print(" ");
  //Serial.print("dt: "); Serial.print(delta);   Serial.print(" ");
  d = d + v*delta;
  //Serial.print("Dist: "); Serial.print(d);   Serial.print(" ");
  }

// complementary filter, sets the combined motion from both accelerometer and gyroscope data
void filter(vector_t acc, vector_t gyr) {
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    //angleX += (gyr.x / GYROSCOPE_SENSITIVITY) * dt/1000.0f; // Angle around the X-axis
    //angleY += (gyr.y / GYROSCOPE_SENSITIVITY) * dt/1000.0f; // Angle around the Y-axis
    //angleZ += (gyr.z / GYROSCOPE_SENSITIVITY) * dt/1000.0f; // Angle around the Z-axis
    // Compensate for drift with accelerometer data if !bullshit
    int forceMagnitudeApprox = fabs(acc.x) + fabs(acc.y) + fabs(acc.z);
    if (forceMagnitudeApprox > 4 && forceMagnitudeApprox < 64)
    {
        float pitchAcc, rollAcc, turnAcc;
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f(acc.z, acc.y) * 180.0f / M_PI;
        angleX = angleX * 0.8f + pitchAcc * 0.20f;

        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(acc.x, acc.z) * 180.0f / M_PI;
        angleY = angleY * 0.8f + rollAcc * 0.2f;

        // Turning around the Z axis results in a vector on X&Y-axis
        //turnAcc = atan2f(acc.y, acc.x) * 180.0f / M_PI;
        //angleZ = angleZ * 0.8f + turnAcc * 0.2f;
        // TODO the orientation can be wrong here!!
    }
}



void setupSensor()
{
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
    Serial.begin(115200);

    while (!Serial) {
        delay(1); // will pause Zero, Leonardo, etc until serial console opens
    }

    Serial.println("LSM9DS1 data read demo");

    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin())
    {
        Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");

    // helper to just set the default scaling we want, see above!
    setupSensor();
    start_time = millis();
}

void loop() 
{
    unsigned long current_time = millis();
    dt = current_time - last_time;
    last_time = current_time;
    lsm.read();  /* ask it to read in the data */

    /* Get a new sensor event */
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp);

    // accel is inverted!!
    //Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    //Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    //Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.print(" m/s^2 ");
    //Serial.println("");

    //Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    //Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    //Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.print(" gauss");
    //Serial.println("");

    //Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
    //Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
    //Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.print(" dps");
    //Serial.println("");
    if (current_time - start_time>1000)
      computeDistance(a.acceleration.x,dt/1000.0,angleY);
    Serial.print("distance: "); Serial.print(d);   Serial.print("    ");
    
    filter(a.acceleration, g.gyro);
    //Serial.print("Angle X: "); Serial.print(d);   Serial.print(" dps");
    //Serial.print("\tY: "); Serial.print(angleY);      Serial.print(" dps");
    //Serial.print("\tZ: "); Serial.print(angleZ);      Serial.print(" dps");

    
    Serial.println();
    delay(1);
}
