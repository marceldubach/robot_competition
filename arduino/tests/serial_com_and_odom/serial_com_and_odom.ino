/*
 * Run this script together with 'serial_com_and_odom.py'
 * Implementation of odometry (Hall sensors + IMU anguar rate)
 * (no libraries)
 * 28.12.2020
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
float rotV;

// definition of an IMU object with associated variables
MPU6050 IMU;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float gyro_sf = 131.00; //[LSB/(°/s)] gain at ±250 configuration DEFAULT ONE
float gyro_mean = 0.86; // mean noise on gyroscope gz lecture, averaged over 1000 data points 
float omega;
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


double x = 0.5;
double y = 0.5;
double theta = 0;

int state;

int cmdRight;
int cmdLeft;

bool enableMotors;

Motors WheelMotors;

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
      double cLeft = receive_msg["cmd"][0];
      double cRight = receive_msg["cmd"][1];
      state = s;
      cmdLeft = cLeft;
      cmdRight= cRight;

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
      position.add(omega_mean);
      position.add(dt_sum);
      cnt = 0;
      omega_mean = 0;
      dt_sum = 0;
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
      digitalWrite(enableRight, LOW);                
      digitalWrite(enableLeft, LOW);
      break;

    case 1:
      // turn motors on
      digitalWrite(enableRight, HIGH);                
      digitalWrite(enableLeft, HIGH);

      // set motor speed (fixed speed)
      analogWrite(pwmRight, 255-cmdRight);
      analogWrite(pwmLeft, cmdLeft);

      omega = 0;
      omega_m1 = 0;

      if (millis()-t0>20){
        // read motorspeeds
        speedRight = analogRead(avSpeedRight);
        speedLeft = analogRead(avSpeedLeft);
  
        dt = (float) (millis() - t0)/1000; // TODO check if this cast works
        t0 = millis();
        dt_sum += dt;
        IMU.getRotation(&gx, &gy, &gz);
        /*if (omega_m1==0){
          omega_m1 = omega;
          omega = gz/gyro_sf - gyro_mean;
        } else {
          omega_m1 = gz/gyro_sf - gyro_mean;
          omega = gz/gyro_sf - gyro_mean;
        }
        */
        omega = gz/gyro_sf - gyro_mean;
                
        //acc_x = ax/acc_sf - acc_mean;
        omega_mean += omega;
        cnt++;
  
        // rescale wheel speed to rad/s
        wLeft = ((speedLeft - 415.00)/415.00)*6.25;
        wRight = -((speedRight - 415.00)/415.00)*6.25;
        
        // mean forward speed in m/s
        v = (wLeft + wRight)*radius/2;
        
        rotV = omega*3.1415/180; // convert rotation rate to rad/s
  
        x = x + cos(theta)*v*dt;
        y = y + sin(theta)*v*dt;
        theta = theta + omega/180*3.1415*dt;
      }
  }

}
