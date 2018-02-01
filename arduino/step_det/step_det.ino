// 90:A2:DA:F8:37:9B

#include <Wire.h>
#include <SPI.h>
#include <Bridge.h>
#include <HttpClient.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>   // for tan2 & fabs

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


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
#define NN 50
int n=0; float yaw_instant[NN];

//current velocity and distance
float v = 0, d = 0;

//step detector
float step_threshold = 400; float turn_threshold = 400; float last_orient = 0; long last_step_time; long last_turn_time; bool first_stp = true;
#define STEPSIZE 0.6

//position x/y
//should be the same as in html,width=7.7,height=6; right turn=-90
float pos[2] = {0.1,4};
float test_pos = 0;
float orient_est = 0;
#define INITANGLE 30

void Angle_Avg(float ang_x) {
  n++;
  yaw_instant[n%NN] = ang_x;
  
  float sum_aX; 

  if(n>NN){
    for(int i=0; i<NN; i++) {
      sum_aX += yaw_instant[i];
      
    }
  }
  yaw = sum_aX / NN;
  return;

}


void setupSensor()
{
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);


    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);


    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}


void setup() 
{
    //Serial.begin(115200);
    Bridge.begin();
//    while (!Serial) {
//        delay(1); // will pause Zero, Leonardo, etc until serial console opens
//    }

    // Try to initialise and warn if we couldn't detect the chip
    if (!lsm.begin())
    {
        while (1);
    }
    //Serial.println("Found LSM9DS1 9DOF");
    pinMode(4, INPUT_PULLUP);
    // helper to just set the default scaling we want, see above!
    setupSensor();
    start_time = millis();
    last_turn_time = millis();
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
    if (current_time - start_time>1000){
      //Serial.println(g.gyro.z);
      //Step_detector: accel.z value peaks at every step => detect peaks and add one step;  tbd: set the threshold adaptive (roll/yaw correction over an average(?))
      step_threshold = 12;
      turn_threshold = 130;
      Angle_Avg(g.gyro.z);

      if ( digitalRead(4) == LOW) {
        pos[0] = 0.1;
        pos[1] = 4;
        orient_est = 0;
        send_orient();
        delay(100);
      }
      
      if (yaw > turn_threshold && millis() - last_turn_time > 800) {

        last_turn_time = millis();
        last_step_time = millis();
        orient_est +=90;
          //Serial.println("Turn +");
          send_orient();
                
      }
      if (yaw < -turn_threshold && millis() - last_turn_time > 800) {

        last_turn_time = millis();
         last_step_time = millis();
        orient_est -=90;
          //Serial.println("Turn -");
          send_orient();
                
      }      
      if (a.acceleration.z > step_threshold && millis() - last_step_time > 500) {

        last_step_time = millis();
        execute_step();
        //  Serial.println("Step");
                
      }

    }
}


void execute_step() {
  test_pos += STEPSIZE;

  //last_turn_time = millis();

  pos[0] += cos(orient_est/180*PI) * STEPSIZE;
  pos[1] += sin(orient_est/180*PI) * STEPSIZE;
  
  HttpClient client;

  // Make a HTTP request:
  //error checking
  String url = "http://192.168.1.100:8000/map/changePos/" + String(pos[0])+"&"+String(pos[1])+"&"+String(orient_est);
  client.get(url);
  return;
}

void send_orient(){
  HttpClient client;

  // Make a HTTP request:
  //error checking
  String url = "http://192.168.1.100:8000/map/changePos/" + String(pos[0])+"&"+String(pos[1])+"&"+String(orient_est);
  client.get(url);
  return;
  }

