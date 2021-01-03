/*
 * Run this script together with 'state_machine_w_KF.py'
 * Experimental: 
 *  - Add state trantions for returning to home and catching bottle
 * Todo: 
 *  - Add map for Obstacles and WP-navigation to avoid obstacles
 *  - Add movement to grab bottles
 *  date: 3.1..2020
 */

#include "ArduinoJson.h"
//#include "Motors.h"
#include "ArduinoJson.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"
#include "VelCtrl.h"

#define PI 3.1415

byte dutyCycle;

// timestep variables (for integration of IMU)
unsigned long t0 = 0;

float dt;
float v = 0;
float omega_rad;
// definition of an IMU object with associated variables
MPU6050 IMU;
int16_t gx, gy, gz;

#define avSpeedRight A1 // Hall sensor reading
#define avSpeedLeft A3  // Hall sensor reading
float speedRight = 0;
float speedLeft = 0;

float wLeft = 0;
float wRight = 0;

float radius = 0.04;
float omega_deg;

float gyro_sf = 131.00; //[LSB/(°/s)] gain at ±250 configuration DEFAULT ONE
float gyro_mean = 0.55; // mean noise on gyroscope gz lecture, averaged over 1000 data points

bool time_elapsed = false;
//int16_t ax, ay, az;
//int16_t mx, my, mz;

//float acc_sf = 16384; // [LSB/g] gain at ±2 configuration DEFAULT ONE
//float acc_mean = -0.04; // mean noise on accelerometre ax lecture, averaged over 1000 data points
//float acc_x;

// wheel motor ports

//MotorRight ports
byte enableRight(51);
byte pwmRight(3); // changed from define
//#define avSpeedRight A1 // Hall sensor reading

//MotorLeft ports
byte enableLeft(50);
byte pwmLeft(2); // changed from define
//#define avSpeedLeft A3 // Hall sensor reading

// estimated position of the robot
double x = 1;
double y = 1;
double theta = 0;

// references to waypoint (goal)
//double heading_ref = 0;
//double dist = 0;

// variables for serial communication
int macro_state;
enum macro_states
{
  STARTING,
  MOVING,
  OBSTACLE,
  CATCH,
  RETURN,
  EMPTY,
  FINISH
};

enum catch_states
{
  TRACK_WP,
  LOWER,
  APPROACH_BOTTLE,
  CLOSE,
  RAISE,
  OPEN
};
int catch_state = 0; // state for catching bottle
unsigned long t_catch = 0;

enum empty_states
{
  OPEN_DOOR,
  SHAKE,
  CLOSE_DOOR
};
int empty_state = 0;
int cnt_shakes = 0;
bool cam_is_up = false;
unsigned long t_empty = 0;

// braitenberg
// pin declaration for the braitenberg vehicle
uint8_t trigger[7] = {25, 23, 27, 29, 31, 33, 35};
uint8_t echo[7] = {24, 22, 26, 28, 30, 32, 34};

int n_US = 7;
int idx_us = 0;
double distances[] = {100, 100, 100, 100, 100, 100, 0};
int threshold[] = {0, 0, 0, 0, 0, 0, 0};
int weight_left[] = {700, 300, 400, -1200, -800, -400, -100};
int weight_right[] = {1000, 400, -400, -300, -600, 400, 300};
const double maxdist = 50;
unsigned long maxPulseIn = 3000; // 50 cm range
unsigned long duration;
//initialize distances at values bigger than the threshold

#define claw_trigger 37
#define claw_echo 36

unsigned long claw_dist = 100;
int cntBottles = 0;

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

void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  mainServo.attach(11, 400, 2550); //400us-2550us DFROBOT high torque
  microLeft.attach(10, 900, 2100); // (pin, min, max) // for HC-82 left
  microRight.attach(9, 900, 2100); // (pin, min, max) // for HC-82 right
  camServo.attach(8, 750, 2250);   // (pin, min, max) // for camshaft servo
  //backDoor.attach(7,750,2250);   // (pin, min, max) // for back door

  for (int i = 0; i < n_US; i++)
  { // set up Echo and Trigger pins
    pinMode(trigger[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  pinMode(claw_trigger, OUTPUT); // for the claw
  pinMode(claw_echo, INPUT);

  pinMode(avSpeedRight, INPUT); //analog input -> average speed motorRight
  pinMode(avSpeedLeft, INPUT);  //analog input -> average speed motorLeft
  pinMode(pwmRight, OUTPUT);    //PWM pin motorRight
  pinMode(enableRight, OUTPUT); //enable pin motorRight
  pinMode(pwmLeft, OUTPUT);     //PWM pin motorLeft
  pinMode(enableLeft, OUTPUT);  //enable pin motorLeft
  digitalWrite(enableRight, LOW);
  digitalWrite(enableLeft, LOW);

  Serial.begin(38400);
  Serial.setTimeout(50);
  while (!Serial)
    continue;
  IMU.initialize();
  while (!IMU.testConnection())
    continue;

  t0 = millis();
  enableMotors = false;
  macro_state = STARTING;
  Serial.println("ready"); // Arduino setup completed
}

void loop()
{
  String line = Serial.readStringUntil('\n');
  DeserializationError error = deserializeJson(receive_msg, line);
  if (error)
  {
    Serial.println(line);
  }
  else
  {
    // read the obtained string
    int old_state = macro_state;

    // python prints the state if it has changed
    if (receive_msg.containsKey("state"))
    {
      macro_state = receive_msg["state"];
    }
    if (receive_msg.containsKey("ref"))
    {
      ref_x = receive_msg["ref"][0];
      ref_y = receive_msg["ref"][1];
    }
    // reset the pose if a KF update is done in python
    if (receive_msg.containsKey("pose"))
    {
      x = receive_msg["pose"][0];
      y = receive_msg["pose"][1];
      theta = receive_msg["pose"][2];
    }
    // STATE UPDATE

    // if the state was set to CATCH: initialize micro_state and timer
    if ((macro_state == CATCH) && (old_state != CATCH))
    {
      t_catch = millis();
      catch_state = TRACK_WP;
    }
    // if state was set to RETURN: nothing special to do
    if (macro_state == RETURN)
    {
      time_elapsed = true;
    }
    // if state was set to EMPTY:  initialize  micro state and timer (only once)
    if ((macro_state == EMPTY) && (old_state != EMPTY))
    {
      t_empty = millis();
      empty_state = OPEN_DOOR;
    }

    const int capacity = 200;
    StaticJsonDocument<capacity> send_msg;

    send_msg["state"] = macro_state;
    send_msg["nBot"] = cntBottles;
    //send_msg["cnt"] = cnt_shakes; // counter for camshaft movements
    JsonArray position = send_msg.createNestedArray("pos");
    position.add(x);     //[m]
    position.add(y);     //[m]
    position.add(theta); // remember to put [rad/s]

    // info to add in the json document for Kalman filter
    JsonArray info = send_msg.createNestedArray("info");
    info.add(v);         //[m/s]
    info.add(omega_rad); //[rad/s]
    info.add(dt);        //[s]

    JsonArray reference = send_msg.createNestedArray("ref");
    reference.add(ref_x); //[m]
    reference.add(ref_y); //[m]

    JsonArray command = send_msg.createNestedArray("cmd");
    command.add(cmdLeft);
    command.add(cmdRight);

    /*
    JsonArray dist = send_msg.createNestedArray("dist");
    dist.add(distances[0]);
    dist.add(distances[1]);
    dist.add(distances[2]);
    dist.add(distances[3]);
    dist.add(distances[4]);
    dist.add(distances[5]);
    dist.add(distances[6]);
    dist.add(duration);
    */
    serializeJson(send_msg, Serial);

    Serial.println();
  }

  // read ultrasonic sensors
  digitalWrite(trigger[idx_us], LOW);
  delayMicroseconds(5);
  digitalWrite(trigger[idx_us], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger[idx_us], LOW);
  duration = pulseIn(echo[idx_us], HIGH, maxPulseIn);
  distances[idx_us] = (double)((duration / 2) / 29.1);
  if (distances[idx_us] == 0)
  {
    distances[idx_us] = 1000; // set the distance to 10m otherwise
  }
  if (distances[idx_us] < maxdist)
  {
    threshold[idx_us] = 1;
  }
  else
  {
    threshold[idx_us] = 0;
  }
  idx_us = idx_us + 1;
  if (idx_us == 7)
  {
    idx_us = 0;
  }

  // simulation: position changes over time...
  bool foundObstacle = false;
  switch (macro_state)
  {
  case STARTING:
    // turn motors off
    enableMotors = false;

    // initialize position of servos;
    mainServo.write(160);
    microLeft.write(80);
    microRight.write(120);
    backDoor.write(160);
    break;

  case MOVING: // moving
    for (int i = 0; i < n_US; i++)
    {
      if (distances[i] < maxdist)
      {
        foundObstacle = true;
      }
    }
    if (foundObstacle)
    {
      macro_state = OBSTACLE; // if foundObstacle, switch to state OBSTACLE
    }
    else
    {
      // compute motor speeds
      calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y);

      // turn motors on
      enableMotors = true;
    } // end else obstacle found
    break;

  case OBSTACLE:
    // TODO: implement motor commands and braitenberg here
    for (int i = 0; i < n_US; i++)
    {
      if (threshold[i] == 1)
      {
        foundObstacle = true;
      }
    }
    if (foundObstacle == false)
    {
      macro_state = MOVING; // change to state moving again
    }
    else
    {
      for (int i = 0; i < n_US; i++)
      {
        cmdLeft = 128 + threshold[i] * weight_left[i] / distances[i];
        cmdRight = 128 + threshold[i] * weight_right[i] / distances[i];
      }
    }
    break;

  case CATCH: // lift bottles
    enableMotors = false;
    break;

  case RETURN:
    // TODO: implement motor commands here
    enableMotors = true;
    calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y);
    break;

  case EMPTY:
    enableMotors = false;
    break;

  case FINISH:
    mainServo.detach();
    microRight.detach();
    microLeft.detach();
    backDoor.detach();
    camServo.detach();
    enableMotors = false;
  }

  // TODO: state machine for obstacle avoidance

  // switch if state is CATCH
  if (macro_state == CATCH)
  {
    switch (catch_state)
    {
    case TRACK_WP:
      calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y);
      enableMotors = true;
      if (sqrt(pow((x - ref_x), 2) + pow((y - ref_y), 2)) < 0.4)
      {
        catch_state = LOWER;
        enableMotors = false;
        t_catch = millis();
      }
      break;

    case LOWER:
      mainServo.write(20);
      if (millis() - t_catch > 1000)
      {
        catch_state = APPROACH_BOTTLE;
        t_catch = millis();
      }
      break;

    case APPROACH_BOTTLE:
      cmdLeft = 150;
      cmdRight = 150;
      if (millis() - t_catch > 5000)
      {
        catch_state = CLOSE;
        t_catch = millis();
      }
      break;

    case CLOSE:
      microLeft.write(40);
      microRight.write(160);
      // TODO: Read ultrasonic sensor + increase counter
      if (millis() - t_catch > 1000)
      {
        digitalWrite(claw_trigger, LOW);
        delayMicroseconds(5);
        digitalWrite(claw_trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(claw_trigger, LOW);
        unsigned long claw_duration = pulseIn(claw_echo, HIGH, 3000); // 50 cm timeOut
        claw_dist = (double)((claw_duration / 2) / 29.1);
        if (claw_dist == 0)
        {
          claw_dist = 100;
        }
        if (claw_dist < 30)
        {
          cntBottles++;
        }
        catch_state = RAISE;
        t_catch = millis();
      }
      break;

    case RAISE:
      mainServo.write(160);
      if (millis() - t_catch > 2000)
      {
        t_catch = millis();
        catch_state = OPEN;
      }
      break;

    case OPEN:
      microLeft.write(80);
      microRight.write(120);
      if (millis() - t_catch > 1000)
      {
        macro_state = MOVING;
      }
      break;

    default:
      mainServo.write(160);
      break;
    }
  }

  if (macro_state == EMPTY)
  {
    switch (empty_state)
    {
    case OPEN_DOOR:
      backDoor.write(60);
      if (millis() - t_empty > 500)
      {
        t_empty = millis();
        empty_state = SHAKE;
        cnt_shakes = 0;
      }
      break;

    case SHAKE:
      if (cam_is_up)
      {
        camServo.write(0); // turn servo down
      }
      else
      {
        camServo.write(70); // turn servo up
      }
      if (millis() - t_empty > 500)
      {
        t_empty = millis();
        // toggle cam_is_up state
        if (cam_is_up == true)
        {
          cam_is_up = false;
          cnt_shakes = cnt_shakes + 1;
          if (cnt_shakes >= 2)
          {
            camServo.write(0);
            empty_state = CLOSE_DOOR;
            t_empty = millis();
          }
        }
        else
        {
          cam_is_up = true;
        }
      }
      break;

    case CLOSE_DOOR:
      backDoor.write(160);
      if (millis() - t_empty > 500)
      {
        // if not homing:
        if (!time_elapsed)
        {
          // if time has not elapes yet, change to MOVING again
          macro_state = MOVING;
        }
        else
        {
          // if time has elapsed, change to FINISHED
          macro_state = FINISH;
        }
      }

    default:
      backDoor.write(160); // backdoor closed
    }
  }
  set_Commands(enableMotors, cmdRight, cmdLeft, pwmRight, pwmLeft, enableRight, enableLeft);

  if ((enableMotors) && (millis() - t0 > 20))
  {
    // Do odometry each 20 ms (independent of state!
    dt = (float)(millis() - t0) / 1000;
    t0 = millis();

    //the following function does not work for some reason
    //update_Odometry(gz, x, y, theta, v, omega_rad, dt);

    // read motorspeeds
    speedRight = analogRead(avSpeedRight);
    speedLeft = analogRead(avSpeedLeft);

    // rescale wheel speed to rad/s
    wLeft = ((speedLeft - 415.00) / 415.00) * 6.25;
    wRight = -((speedRight - 415.00) / 415.00) * 6.25;

    // mean forward speed in m/s
    v = (wLeft + wRight) * radius / 2;

    // gyro angular rate
    IMU.getRotation(&gx, &gy, &gz);
    omega_deg = gz / gyro_sf - gyro_mean; // deg/sec
    omega_rad = omega_deg * 3.1415 / 180; // rad/s

    // update the position
    x = x + cos(theta) * v * dt;
    y = y + sin(theta) * v * dt;
    theta = theta + omega_rad * dt; // theta is in radians
    if (theta > 2 * PI)
    {
      theta = theta - 2 * PI; // theta in [0,2*PI]
    }
    else if (theta < 0)
    {
      theta = theta + 2 * PI;
    }
  } // end odometry
}
