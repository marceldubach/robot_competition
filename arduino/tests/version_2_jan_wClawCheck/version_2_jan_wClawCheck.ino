/*
 * Run this script together with 'version_30_dec.py'
 * Implementation: 
 *  - odometry (Hall sensors + IMU anguar rate) (no position feedback yet)
 *  - tracking of position commands (waypoints) -> to be improved
 *  - braitenberg -> weights might need to be adjusted
 *  date: 02.01.2021
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
//int16_t ax, ay, az;
int16_t gx, gy, gz;
//int16_t mx, my, mz;
float gyro_sf = 131.00; //[LSB/(°/s)] gain at ±250 configuration DEFAULT ONE
float gyro_mean = 0.55; // mean noise on gyroscope gz lecture, averaged over 1000 data points

float omega_mean = 0;

float acc_sf = 16384;   // [LSB/g] gain at ±2 configuration DEFAULT ONE
float acc_mean = -0.04; // mean noise on accelerometre ax lecture, averaged over 1000 data points
float acc_x;

// wheel motor ports

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

// references to waypoint (goal)
double heading_ref = 0;
double dist = 0;

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
  APPROACH_BOTTLE,
  LOWER,
  CLOSE,
  RAISE,
  OPEN
};
int catch_state = 0; // state for catching bottle
unsigned long t_catch = 0;

// pin declaration for the braitenberg vehicle
uint8_t trigger[7] = {25, 23, 27, 29, 31, 33, 35};
uint8_t echo[7] = {24, 22, 26, 28, 30, 32, 34};

int n_US = 7;
int idx_us = 0;
double distances[] = {100, 100, 100, 100, 100, 100, 0};
int threshold[] = {0, 0, 0, 0, 0, 0, 0};
int weight_left[] = {700, 300, 400, -1200, -800, -400, -100};
int weight_right[] = {1000, 400, -400, -300, -600, 400, 300};
const double maxdist = 1;        // TODO change this back to 50
unsigned long maxPulseIn = 3000; // PulseIn timeOut => 50 cm range
unsigned long duration;

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

// define pins  and variables for the Claw ultrasensors
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

  Serial.begin(38400);   // initialise the Serial communication
  Serial.setTimeout(50); // set the timout to 50 ms

  while (!Serial)
    continue;
  IMU.initialize();
  while (!IMU.testConnection())
    continue;

  t0 = millis();
  enableMotors = false; // set motors off at start
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
    int old_state = macro_state;
    // read the string obtained
    if (receive_msg.containsKey("state"))
    {
      macro_state = receive_msg["state"];
    }
    if (receive_msg.containsKey("ref"))
    {
      ref_x = receive_msg["ref"][0];
      ref_y = receive_msg["ref"][1];
    }
    if (receive_msg.containsKey("pose"))
    {
      x = receive_msg["pose"][0];
      y = receive_msg["pose"][1];
      theta = receive_msg["pose"][2];
    }

    // change the state
    if ((macro_state == CATCH) && (old_state != CATCH))
    {
      t_catch = millis();
      catch_state = APPROACH_BOTTLE;
    }
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
    info.add(dt);        //[rad]
    //info.add(acc); //[m/s^2]

    JsonArray reference = send_msg.createNestedArray("ref");
    reference.add(ref_x);
    reference.add(ref_y);

    JsonArray command = send_msg.createNestedArray("cmd");
    command.add(cmdLeft);
    command.add(cmdRight);
    //command.add(heading_ref);
    //command.add(dist);

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
      macro_state = OBSTACLE;
    }
    else
    {
      // update the motor speed
      heading_ref = atan2((ref_y - y), (ref_x - x)); // TODO maybe rescale to [0,2PI]

      dist = sqrt(pow((ref_y - y), 2) + pow((ref_x - x), 2));

      if (fabs(heading_ref - theta) < 0.3)
      {
        if (dist > 0.5)
        {
          cmdLeft = 240;
          cmdRight = 240;
        }
        else
        {
          cmdLeft = 160;
          cmdRight = 160;
        }
      }
      else
      {
        if (heading_ref - theta > 0)
        {
          // turn left
          if (heading_ref - theta < 0.5)
          {
            cmdRight = 140;
            cmdLeft = 116;
          }
          else
          {
            cmdRight = 160;
            cmdLeft = 96;
          }
        }
        else
        {
          // turn right
          if (theta - heading_ref < 0.5)
          {
            cmdRight = 116;
            cmdLeft = 140;
          }
          else
          {
            cmdRight = 96;
            cmdLeft = 160;
          }
        } // end else turn right
      }   // end else deltatheta>0.3

      // turn motors on
      enableMotors = true;
    } // end else obstacle found
    break;

  case OBSTACLE:
    // implemented motor commands and braitenberg here
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
    switch (catch_state) // Open loop procedure : 
    {
    case APPROACH_BOTTLE: 
      // position claw
      mainServo.write(30);
      microLeft.write(50);
      microRight.write(150);

      // Enable motors and move 10 cm
      enableMotors = true;
      cmdLeft = 140;
      cmdRight = 140;

      if (millis() - t_catch > 1000)
      {
        catch_state = LOWER;
        enableMotors = false;
        t_catch = millis();
      }
      break;

    case LOWER:
      mainServo.write(15);
      if (millis() - t_catch > 1000)
      {
        catch_state = CLOSE;
        t_catch = millis();
      }
      break;

    case CLOSE:
      microLeft.write(40);
      microRight.write(160);
      if (millis() - t_catch > 1000)
      {
        // read Claw US
        digitalWrite(claw_trigger, LOW);
        delayMicroseconds(5);
        digitalWrite(claw_trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(claw_trigger, LOW);
        unsigned long claw_duration = pulseIn(claw_echo, HIGH, 3000); // 50 cm timeOut
        claw_dist = (double)((claw_duration / 2) / 29.1);

        if (claw_dist == 0) // if PulseIn timed out
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
        macro_state = 3;
      }

    default:
      mainServo.write(160);
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
        macro_state = MOVING;
      }

    default:
      backDoor.write(160); // backdoor closed
    }
  }

  // set motor commands if they are Enabled
  if (enableMotors)
  {
    digitalWrite(enableRight, HIGH);
    digitalWrite(enableLeft, HIGH);
    // saturation
    if (cmdRight > 240)
    {
      cmdRight = 240;
    }
    else if (cmdRight < 15)
    {
      cmdRight = 15;
    }
    if (cmdLeft > 240)
    {
      cmdLeft = 240;
    }
    else if (cmdLeft < 15)
    {
      cmdLeft = 15;
    }
    // set motor speed (fixed speed)
    analogWrite(pwmRight, 255 - cmdRight);
    analogWrite(pwmLeft, cmdLeft);
  }
  else
  {
    digitalWrite(enableRight, LOW);
    digitalWrite(enableLeft, LOW);
  }

  if ((enableMotors) && (millis() - t0 > 20))
  {
    // time interval
    dt = (float)(millis() - t0) / 1000;
    t0 = millis();

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
  }
}
