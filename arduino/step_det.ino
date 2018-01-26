#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>   // for tan2 & fabs

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
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
float yaw;
sensors_vec_t   orientation;

//for moving average
#define N 50
int n=0; float yaw_instant[N];

//current velocity and distance
float v = 0, d = 0;

//step detector
float step_threshold = 400; float turn_threshold = 400; float last_orient = 0; long last_step_time; long last_turn_time; bool first_stp = true;
#define STEPSIZE 0.65

//position x/y
float pos[2] = {0,0};
float test_pos = 0;
float orient_est = 180;
#define INITANGLE 30

#include <SPI.h>
#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <ArduinoJson.h>

BridgeServer server;

JsonObject& prepareResponse(JsonBuffer& jsonBuffer, double x, double y) {
  JsonObject& root = jsonBuffer.createObject();

  JsonArray& digitalValues = root.createNestedArray("position");
  //write value 100
    digitalValues.add(x, y);
  return root;
}


void writeResponse(BridgeClient& client, JsonObject& json) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  json.prettyPrintTo(client);
}

void Angle_Avg(float ang_x) {
  n++;
  yaw_instant[n%N] = ang_x;
  
  float sum_aX; 

  if(n>N){
    for(int i=0; i<N; i++) {
      sum_aX += yaw_instant[i];
      
    }
  }
  yaw = sum_aX / N;
  
  //Serial.print("Yaw: "); Serial.print(yaw);   Serial.println(" ° ");

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
    last_turn_time = millis();

      Bridge.begin();
  //port 5555
  server.noListenOnLocalhost();
  server.begin();
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



    //Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
    //Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
    //Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.print(" dps");
    //Serial.println("");

    if (current_time - start_time>1000){
      //Step_detector: accel.z value peaks at every step => detect peaks and add one step;  tbd: set the threshold adaptive (roll/yaw correction over an average(?))
      step_threshold = 12;
      turn_threshold = 130;
      Angle_Avg(g.gyro.z);
      
//      Serial.print(orientation.heading); Serial.print("   ");
//      Serial.print(a.acceleration.z); Serial.print("   ");


        
  
//  if (millis() - last_turn_time > 1000) {
//
//    if ((yaw >= 0 && last_orient >= 0) || (yaw <= 0 && last_orient <= 0)) {
//
//      if (yaw - last_orient > 45) {
//        //Serial.println("Changed + ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est += 90;
//      }
//      else if (last_orient - yaw < -45) {
//        //Serial.println("Changed - ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est -= 90;        
//      }
//    }
//    else if (yaw < 0 && last_orient > 0) {
//     if (last_orient >= 90 && yaw < -90) {
//      if (yaw + 360 - last_orient > 45) {
//        //Serial.println("Changed + ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est += 90;        
//      }
//     }
//     else if (last_orient < 90 && yaw > -90) {
//      if (last_orient - yaw > 45) {
//        //Serial.println("Changed - ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est -= 90;         
//      }
//     }
//    }
//    else if (yaw > 0 && last_orient < 0) {
//     if (last_orient < -90 && yaw >= 90) {
//      if (last_orient + 360 - yaw > 45) {
//        //Serial.println("Changed - ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est -= 90;
//      }
//     }
//     else if (last_orient > -90 && yaw < 90) {
//      if (yaw - last_orient > 45) {
//        //Serial.println("Changed + ");
//        //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
//        //Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
//        orient_est += 90;
//      }
//     }  
//    }
////  if ((yaw-last_orient > 45) && (yaw + 360 - last_orient > 45)) {
////    Serial.println("Changed + ");
////    Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
////    Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
////    orient_est += 90;
////  }
////  else if ((last_orient - yaw > 45) && (last_orient - (yaw - 360)> 45 )) {
////    Serial.println("Changed - ");
////    Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
////    Serial.print("yaw: "); Serial.print(yaw);   Serial.print(" ° ");
////    orient_est -= 90;
////  }
//  last_orient = yaw;
//  last_turn_time = millis();
//  }

      if (yaw > turn_threshold && millis() - last_turn_time > 800) {

        last_turn_time = millis();
        orient_est +=90;
          Serial.println("Turn +");
          //Serial.print("Last: "); Serial.print(last_step_time);   Serial.print(" m; ");
          //Serial.print("Millis: "); Serial.print(millis());   Serial.print(" m; ");
                
      }
      if (yaw < -turn_threshold && millis() - last_turn_time > 800) {

        last_turn_time = millis();
        orient_est -=90;
          Serial.println("Turn -");
          //Serial.print("Last: "); Serial.print(last_step_time);   Serial.print(" m; ");
          //Serial.print("Millis: "); Serial.print(millis());   Serial.print(" m; ");
                
      }      
      if (a.acceleration.z > step_threshold && millis() - last_step_time > 500) {

        last_step_time = millis();
        execute_step();
          Serial.println("Step");
          //Serial.print("Last: "); Serial.print(last_step_time);   Serial.print(" m; ");
          //Serial.print("Millis: "); Serial.print(millis());   Serial.print(" m; ");
                
      }

        if(abs(a.acceleration.z) < step_threshold && millis()-last_step_time>100){
          //ahrs.getOrientation(&orientation);
          //Angle_Avg(orientation.heading);

        }
    }
}


void execute_step() {
  test_pos += STEPSIZE;

  //last_turn_time = millis();
  
 
  float dummy = orient_est;

    pos[0] += cos(dummy/180*PI) * STEPSIZE;
    pos[1] += sin(dummy/180*PI) * STEPSIZE;

  //Serial.print("yaw: "); Serial.print(yaw);  Serial.print("  ");  
  //Serial.print("X: "); Serial.print(pos[0]);   Serial.print(" m; ");
  //Serial.print("Y: "); Serial.print(pos[1]);   Serial.print(" m; ");
  //Serial.print("last_orient: "); Serial.print(last_orient);   Serial.print(" ° ");
  //Serial.println(yaw);Serial.print("   ");
  //Serial.print("Yaw: "); Serial.print(avg_yaw_t);   Serial.print(" °");
  //Serial.println();

   // listen for incoming clients
  BridgeClient client = server.accept();
  
  if (client) {
      String command = client.readString();
      command.trim();
      Serial.println(command);
        // Use http://arduinojson.org/assistant/ to
        // compute the right size for the buffer
        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& json = prepareResponse(jsonBuffer, pos[0], pos[1]);
        writeResponse(client, json);
      delay(1);
      client.stop();
}
}
