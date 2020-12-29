/*
 * Run this script together with 'position_control.py'
 * Implementation: 
 *  - odometry (Hall sensors + IMU anguar rate)
 *  - tracking of position commands (waypoints)
 *  - not tested on arduino yet
 *  -(no libraries for .cpp)
 *  
 *  date: 29.12.2020
 */

#include "ArduinoJson.h"
#include "Motors.h"
#include "ArduinoJson.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

byte dutyCycle;

// timestep variables (for integration of IMU)
unsigned long t0 = 0;
float dt;
float dt_sum;

float speedRight = 0;
float speedLeft = 0;
float v = 0;
float wLeft = 0;
float wRight = 0;

float radius = 0.04;
float omega_deg;
float omega_rad;
// definition of an IMU object with associated variables
MPU6050 IMU;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float gyro_sf = 131.00; //[LSB/(°/s)] gain at ±250 configuration DEFAULT ONE
float gyro_mean = 0.86; // mean noise on gyroscope gz lecture, averaged over 1000 data points 

float omega_m1;
float omega_mean = 0;
int cnt = 0;

float acc_sf = 16384; // [LSB/g] gain at ±2 configuration DEFAULT ONE
float acc_mean = -0.04; // mean noise on accelerometre ax lecture, averaged over 1000 data points 
float acc_x;

// Motor Ports
//MotorRight ports
#define enableRight 51
#define pwmRight 3
#define avSpeedRight A1 // Hall sensor reading

//MotorLeft ports
#define enableLeft 50
#define pwmLeft 2
#define avSpeedLeft A3 // Hall sensor reading

// estimated position of the robot
double x = 0.5;
double y = 0.5;
double theta = 0;

double heading_ref = 0;
double dist = 0;

// variables for serial communication
int state;
// set default values
double ref_x = 0.5;
double ref_y = 0.5;
int cmdRight = 128;
int cmdLeft = 128;

bool enableMotors;

StaticJsonDocument<200> receive_msg;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  pinMode(avSpeedRight, INPUT); //analog input -> average speed motorRight
  pinMode(avSpeedLeft, INPUT); //analog input -> average speed motorLeft
  pinMode(pwmRight, OUTPUT); //PWM pin motorRight
  pinMode(enableRight, OUTPUT); //enable pin motorRight
  pinMode(pwmLeft, OUTPUT); //PWM pin motorLeft
  pinMode(enableLeft, OUTPUT); //enable pin motorLeft
  digitalWrite(enableRight, LOW);                
  digitalWrite(enableLeft, LOW);

  Serial.begin(38400);
  while (!Serial) continue;
  IMU.initialize();
  while(!IMU.testConnection()) continue;

  t0 = millis();
  enableMotors = false;
  state = 0;
  Serial.println("ready"); // Arduino setup completed

}

void loop() {
if (Serial.available()>0){
    String line = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(receive_msg, line);
    if (error){
      Serial.println("Deserialize failed");
    } else {
      int s = receive_msg["state"];
      double ref_x = receive_msg["ref"][0];
      double ref_y = receive_msg["ref"][1];
      state = s;

      const int capacity=200;
      StaticJsonDocument<capacity>send_msg;
  
      send_msg["state"] = state;
      JsonArray position = send_msg.createNestedArray("pos");
      //String x_string = String(x,1);
      //String y_string = String(y,1);
      //String theta_string = String(theta,2);
      omega_mean = omega_mean/cnt;
      
      position.add(x);
      position.add(y);
      position.add(theta*180/3.14);

      JsonArray reference = send_msg.createNestedArray("ref");
      reference.add(ref_x);
      reference.add(ref_y);

      JsonArray command = send_msg.createNestedArray("cmd");
      command.add(cmdLeft);
      command.add(cmdRight);
      serializeJson(send_msg, Serial);

      Serial.println();
    }
    
  }

  // simulation: position changes over time...
  switch (state){
    case 0:
      // turn motors off
      enableMotors = false;
      break;

    case 1:
      // update the motor speed
      heading_ref = atan2((ref_y-y), (ref_x-x));
      dist = sqrt(pow((ref_y-y),2)+pow((ref_x-x),2));

      if (fabs(heading_ref-theta)<0.1){
        if (dist>0.5){
          cmdLeft = 200;
          cmdRight = 200;
        }else{
          cmdLeft = 160;
          cmdRight = 160;
        }
      }else{
        if (heading_ref-theta>0){
          // turn left
          cmdRight = 160;
          cmdLeft = 96;
        } else {
          // turn right
          cmdRight = 96;
          cmdLeft = 160;
        }
      }
      // turn motors on
      enableMotors = true;
      break;

    if (enableMotors){
      digitalWrite(enableRight, HIGH);                
      digitalWrite(enableLeft, HIGH);

      // set motor speed (fixed speed)
      analogWrite(pwmRight, 255-cmdRight);
      analogWrite(pwmLeft, cmdLeft);
    } else {
      digitalWrite(enableRight, LOW);                
      digitalWrite(enableLeft, LOW);
    }
    
    if ((enableMotors) && (millis()-t0>20)){
      // time interval
      dt = (float) (millis() - t0)/1000;
      t0 = millis();
      
      // read motorspeeds
      speedRight = analogRead(avSpeedRight);
      speedLeft = analogRead(avSpeedLeft);
      
      // rescale wheel speed to rad/s
      wLeft = ((speedLeft - 415.00)/415.00)*6.25;
      wRight = -((speedRight - 415.00)/415.00)*6.25;
      
      // mean forward speed in m/s
      v = (wLeft + wRight)*radius/2;
      
      // gyro angular rate
      IMU.getRotation(&gx, &gy, &gz);
      omega_deg = gz/gyro_sf - gyro_mean; // deg/sec
      omega_rad = omega_deg*3.1415/180;   // rad/s
  
      // update the position
      x = x + cos(theta)*v*dt;
      y = y + sin(theta)*v*dt;
      theta = theta + omega_rad*dt;       // theta is in radians
    }
  }

}
