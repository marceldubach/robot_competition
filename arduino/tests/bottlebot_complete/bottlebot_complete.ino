/*
 * Run this script together with 'state_machine_w_KF.py'
 * Final version. Bottles are not always avoided..
 * date: 7.1..2020
 */

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
float gyro_mean = 0.46; // mean noise on gyroscope gz lecture, averaged over 1000 data points

//MotorRight ports
byte enableRight(51);
byte pwmRight(3); // changed from define

//MotorLeft ports
byte enableLeft(50);
byte pwmLeft(2); // changed from define

// estimated position of the robot
double x = 1;
double y = 1;
double theta = 0;

// variables for serial communication
int macro_state;
int old_macro_state;
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
  ROTATE,
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
//uint8_t trigger[7] = {25, 23, 27, 29, 31, 33, 35};
//uint8_t echo[7] = {24, 22, 26, 28, 30, 32, 34};

//int n_US = 7;
//int idx_us = 0;
//double distances[] = {100, 100, 100, 100, 100, 100, 100};
//int threshold[] = {0, 0, 0, 0, 0, 0, 0};
//double weight_left[] = {1000, -400, -300, -1200, -1200, -300, -400}; // old: double
//double weight_right[] = {1000, -400, -500, -1400, -1400, -500, -400}; // double
//double maxdist[] = {30,30,40,70,70,40,30}; // initialize maxdist at less than default distances!

uint8_t trigger[8] = {25, 23, 27, 29, 37, 31, 33, 35};
uint8_t echo[8] = {24, 22, 26, 28, 36,30, 32, 34};
int n_US = 8;
int idx_us = 0;
int threshold[] = {0, 0, 0, 0, 0,  0, 0, 0};
double distances[] = {100, 100, 100, 100,100, 100, 100, 100};
double weight_left[] =  {20, 20, -30, -40, -40, -40, -30, 20};//{20, -10, -40, -40, -40, -40, -40, -10};
double weight_right[] = {20, 20, -30, -40, -40, -40, -30, 20}; //{20, -10, -20, -20, -20, -20, -20, -10};
double fr_dist = 70;
double turn_dist = 30;
double maxdist[] = {30,30,50,fr_dist,fr_dist,fr_dist,50,30};
unsigned long maxPulseIn = 7000; // 50 cm range
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
  backDoor.attach(7,750,2250);   // (pin, min, max) // for back door

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
  Serial.setTimeout(100); //
  while (!Serial)
    continue;
  IMU.initialize();
  while (!IMU.testConnection())
    continue;

  t0 = millis();
  
  macro_state = STARTING;
  enableMotors = false;
  //Serial.println("ready"); // Arduino setup completed
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
    // old_macro_state = macro_state;

    // python prints the state if it has changed
    if (receive_msg.containsKey("state"))
    { 
      int new_state = receive_msg["state"];
      if (new_state == FINISH){
        macro_state = receive_msg["state"]; // do shutdown ALWAYS
        enableMotors = false;
      }
      else if (macro_state != OBSTACLE){ // else change state only if not in obstacle avoidance
        old_macro_state = macro_state; // keep track of the old state!
        macro_state = new_state;
        // if PYTHON sets state to CATCH: initialize micro_state and timer
        if ((macro_state == CATCH) && (old_macro_state != CATCH))
        {
          t_catch = millis();
          catch_state = TRACK_WP;
          enableMotors = true;
        }
        // if PYTHON sets state to RETURN: set time_elapsed to true
        if ((macro_state == RETURN)&&(old_macro_state != RETURN)){
          //time_elapsed = true;
        }
        // if PYTHON sets to EMPTY: initialize substate
        if ((macro_state == EMPTY)&&(old_macro_state != EMPTY)){
          t_empty = millis();
          empty_state = ROTATE;
        }// end else if macro_state != OBSTACLE
      }
    } // end if key "state" contained in message
    
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
   
  }
  // read ultrasonic sensors
  for(int idx_us=0; idx_us<8; idx_us++){
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
      if (distances[idx_us] < maxdist[idx_us])
      {
        threshold[idx_us] = 1;
      }
      else
      {
        threshold[idx_us] = 0;
      }
  }
  // update the variable if there is an obstacle
  bool foundObstacle = false;
  for (int i = 0; i < n_US; i++)
  {
    if (threshold[i] == 1)
    {
      foundObstacle = true;
    }
  }
  
  switch (macro_state)
  {
  case STARTING:
    // turn motors off
    //enableMotors = false;

    // initialize position of servos;
    mainServo.write(160);
    microLeft.write(82);
    microRight.write(120);
    backDoor.write(175);
    break;

  case MOVING: // moving
    if (foundObstacle)
    {
      old_macro_state = macro_state;
      macro_state = OBSTACLE; // if foundObstacle, switch to state OBSTACLE
    }
    else
    {
      // compute motor speeds
      enableMotors = true;
      double del_theta = 0.3;
      calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y,maxdist);
    } // end else obstacle found
    break;

  case OBSTACLE:
    if (foundObstacle == false)
    {
      // get back to old state
      int tmp_state = macro_state;
      if (old_macro_state == CATCH){
        macro_state = MOVING;
      } else {
        macro_state = old_macro_state; // change to previous again
      }
      old_macro_state = tmp_state;
    }
    else
    { 
      cmdLeft = 128;
      cmdRight = 128;
      for (int i = 0; i < n_US; i++)
      {
        cmdLeft += threshold[i] * weight_left[i];// / distances[i];
        cmdRight += threshold[i] * weight_right[i];// / distances[i];
      }
    }
    break;

  case CATCH: // lift bottles
    if (foundObstacle){
      maxdist[2] = 20;
      maxdist[3] = fr_dist;
      maxdist[4] = fr_dist;
      maxdist[5] = fr_dist;
      maxdist[6] = 20;
      old_macro_state = macro_state;
      macro_state = OBSTACLE;
      microLeft.write(82);
      microRight.write(120);
      mainServo.write(160);
      
    }
    break;

  case RETURN:
    microLeft.write(50);
    microRight.write(162);
    mainServo.write(160);
    if (foundObstacle)
    {
      old_macro_state = macro_state;
      macro_state = OBSTACLE; // if foundObstacle, switch to state OBSTACLE
    } else {
      enableMotors = true;
      double del_theta = 0.3;
      calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y,maxdist);
          
    }
    break;

  case EMPTY:
    if (foundObstacle)
    { 
      old_macro_state = macro_state;
      macro_state = OBSTACLE; // if foundObstacle, switch to state OBSTACLE
    }
    break;

  case FINISH:
    mainServo.detach();
    microRight.detach();
    microLeft.detach();
    backDoor.detach();
    camServo.detach();
    break;
  }

  // switch if state is CATCH
  if (macro_state == CATCH)
  {
    switch (catch_state)
    {
    case TRACK_WP:
      /*
      double distance_to_WP = sqrt(pow(x-ref_x,2)+pow(y-ref_y,2));
      double del_theta = 0.3;
      if (distance_to_WP<1){
        del_theta = 0.3+ (1-distance_to_WP);
      }
      */
      maxdist[3] = 0;
      maxdist[4] = 0;
      maxdist[5] = 0;
      calculate_Commands(cmdLeft, cmdRight, x, y, theta, ref_x, ref_y);
      enableMotors = true;
      if (sqrt(pow((x - ref_x), 2) + pow((y - ref_y), 2)) < 0.45)
      {
        catch_state = LOWER;
        enableMotors = false;
        t_catch = millis();
      }
      break;

    case LOWER:
      mainServo.write(19);
      microLeft.write(100);
      microRight.write(100);
      if (millis() - t_catch > 1000)
      {
        catch_state = APPROACH_BOTTLE;
        t_catch = millis();
        enableMotors = true;
      }
      break;

    case APPROACH_BOTTLE:
      cmdLeft = 150;
      cmdRight = 150;
      if (millis() - t_catch > 4000)
      {
        enableMotors = false;
        catch_state = CLOSE;
        t_catch = millis();
      }
      break;

    case CLOSE:
      microLeft.write(50);
      microRight.write(162);
      
      if (millis() - t_catch > 1000)
      {
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
      microLeft.write(85);
      microRight.write(120);
      if (millis() - t_catch > 1000)
      {
        macro_state = MOVING;
        enableMotors = true;
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
    case ROTATE:
      enableMotors = true;
      if (fabs(theta-PI/4)>0.5){
        if (theta<5.0/8*PI){
          cmdLeft = 150;
          cmdRight = 106;
        } else {
          cmdLeft = 106;
          cmdRight = 150;
        }
      } else if(fabs(theta-PI/4)>0.3){
          if (theta>PI/4){
            // turn right
            cmdLeft = 135;
            cmdRight = 121;
          } else{
            cmdLeft = 121;
            cmdRight = 135;  
          }
      } else {
        enableMotors = false;
        empty_state = OPEN_DOOR;
      }
      break;
      
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
        camServo.write(10); // turn servo down
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
        }
        else
        {
          cam_is_up = true;
          cnt_shakes = cnt_shakes + 1;
          if (cnt_shakes >= 4)
          {
            camServo.write(10);
            empty_state = CLOSE_DOOR;
            t_empty = millis();
          }
        }
      }
      break;

    case CLOSE_DOOR:
      backDoor.write(175);
      if (millis() - t_empty > 500)
      {
        macro_state = FINISH; // change to MOVING (always!)
        // if the time is elapsed, the robot will change from MOVING to shutdown in the next loop
      }
      break;

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

  if (!error){
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

    /*
    JsonArray command = send_msg.createNestedArray("cmd");
    command.add(cmdLeft);
    command.add(cmdRight);
    */
    /*
    if (macro_state == OBSTACLE){
      JsonArray dist = send_msg.createNestedArray("dist");
      dist.add(distances[0]);
      dist.add(distances[1]);
      dist.add(distances[2]);
      dist.add(distances[3]);
      dist.add(distances[4]);
      dist.add(distances[5]);
      dist.add(distances[6]);
      dist.add(distances[7]); 
    } else {
      JsonArray reference = send_msg.createNestedArray("ref");
      reference.add(ref_x); //[m]
      reference.add(ref_y); //[m]
    }
    */
    JsonArray dist = send_msg.createNestedArray("dist");
    dist.add(distances[0]);
    dist.add(distances[1]);
    dist.add(distances[2]);
    dist.add(distances[3]);
    dist.add(distances[4]);
    dist.add(distances[5]);
    dist.add(distances[6]);
    dist.add(distances[7]); 
    serializeJson(send_msg, Serial);

    Serial.println();
  }
}
