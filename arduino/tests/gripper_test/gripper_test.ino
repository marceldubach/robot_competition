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
#include "Servo.h"

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

double dx = 0;
double dy = 0;
double dx2 = 0;
double heading_ref = 0;
double dist = 0;

// variables for serial communication
int macro_state = 0;
unsigned long t_microstate = 0;

enum micro_states {LOWER, CLOSE, RAISE, OPEN};
int micro_state = 0;
// set default values
double ref_x = 0.0;
double ref_y = 0.0;
int cmdRight = 128;
int cmdLeft = 128;

bool enableMotors;

StaticJsonDocument<200> receive_msg;

Servo mainServo;  // high torque servo
Servo microLeft;  // micro servo left
Servo microRight; // micro servo right
Servo camServo;   // cam shaft servo
Servo backDoor;   // servo at back door

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  mainServo.attach(11,400,2550); //400us-2550us DFROBOT high torque 
  microLeft.attach(10,900,2100); // (pin, min, max) // for HC-82 left
  microRight.attach(9,900,2100); // (pin, min, max) // for HC-82 right
  camServo.attach(8,750,2250);   // (pin, min, max) // for camshaft servo
  backDoor.attach(7,750,2250);   // (pin, min, max) // for back door

  
  pinMode(avSpeedRight, INPUT); //analog input -> average speed motorRight
  pinMode(avSpeedLeft, INPUT); //analog input -> average speed motorLeft
  pinMode(pwmRight, OUTPUT); //PWM pin motorRight
  pinMode(enableRight, OUTPUT); //enable pin motorRight
  pinMode(pwmLeft, OUTPUT); //PWM pin motorLeft
  pinMode(enableLeft, OUTPUT); //enable pin motorLeft
  digitalWrite(enableRight, LOW);                
  digitalWrite(enableLeft, LOW);

  Serial.begin(38400);
  Serial.setTimeout(50);
  while (!Serial) continue;
  IMU.initialize();
  while(!IMU.testConnection()) continue;

  t0 = millis();
  enableMotors = false;
  macro_state = 0;
  Serial.println("ready"); // Arduino setup completed
}

void loop() {
  String line = Serial.readStringUntil('\n');
  DeserializationError error = deserializeJson(receive_msg, line);
  if (error){
    Serial.println(line); 
  } else {
    int old_state = macro_state;
    macro_state = receive_msg["state"];
    ref_x = receive_msg["ref"][0];
    ref_y = receive_msg["ref"][1];

    if ((macro_state == 2)&&(old_state != 2)){
      t_microstate = millis();
      micro_state = LOWER;
    }
    const int capacity=200;
    StaticJsonDocument<capacity>send_msg;

    send_msg["state"] = macro_state;
    JsonArray position = send_msg.createNestedArray("pos");

    position.add(x); //[m]
    position.add(y); //[m]
    position.add(theta*180/3.1415); // remember to put [rad/s]

    /* // info to add in the json document for Kalman filter
    JsonArray info = send_msg.createNestedArray("info");
    info.add(v); //[m/s]
    info.add(omega_rad); //[rad/s]
    info.add(dt); //[rad]
    info.add(acc); //[m/s^2]
    */
    JsonArray reference = send_msg.createNestedArray("ref");
    reference.add(ref_x);
    reference.add(ref_y);

    JsonArray command = send_msg.createNestedArray("cmd");
    command.add(cmdLeft);
    command.add(cmdRight);
    command.add(heading_ref);
    command.add(dist);
    serializeJson(send_msg, Serial);

    Serial.println();
  }  

  // simulation: position changes over time...
  switch (macro_state){
    case 0:
      // turn motors off
      enableMotors = false;

      // initialize position of servos;
      mainServo.write(160);
      microLeft.write(80);
      microRight.write(120);
      backDoor.write(160);
      break;

    case 1: // moving
      // update the motor speed
      heading_ref = atan2((ref_y-y), (ref_x-x)); // TODO maybe rescale to [0,2PI]

      dist = sqrt(pow((ref_y-y),2)+pow((ref_x-x),2));

      if (fabs(heading_ref-theta)<0.3){
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
          if (heading_ref-theta<0.5){
            cmdRight = 140;
            cmdLeft = 116;
          }else{
            cmdRight = 160;
            cmdLeft = 96;
          }
        } else {
          // turn right
          if (theta-heading_ref<0.5){
            cmdRight = 116;
            cmdLeft = 140;
          } else {
            cmdRight = 96;
            cmdLeft = 160;
          }
        }
      }

      // saturation
      if (cmdRight>255){
        cmdRight = 255;
      } else if (cmdRight<0){
         cmdRight = 0;
      }
      if (cmdLeft>255){
        cmdLeft = 255;
      } else if (cmdLeft<0){
         cmdLeft = 0;
      }

      // turn motors on
      enableMotors = true;
      break;

    case 2: // lift bottles
      enableMotors = false;
      break;

    case 3:
      mainServo.detach();
      microRight.detach();
      microLeft.detach();
      backDoor.detach();
      camServo.detach();
      enableMotors = false;
  }
  if (macro_state==2){
    switch (micro_state){
      case LOWER:
        mainServo.write(15);
        if (millis()-t_microstate>1000){
          micro_state = CLOSE;
          t_microstate = millis();
        }
        break;

      case CLOSE:
        microLeft.write(40);
        microRight.write(160);
        // TODO: Read ultrasonic sensor + increase counter
        if (millis()-t_microstate>1000){
          micro_state = RAISE;
          t_microstate = millis();
        }
        break;

      case RAISE:
        mainServo.write(160);
        if (millis()-t_microstate>2000){
          t_microstate = millis();
          micro_state = OPEN;
        }
        break;

      case OPEN:
        microLeft.write(80);
        microRight.write(120);
        if (millis()-t_microstate>1000){
          macro_state = 3;
        }

      default:
        mainServo.write(160);
      
    }
  }

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
