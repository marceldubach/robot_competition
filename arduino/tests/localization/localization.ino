/*
 * Localization test 27.12.2020
 * includes sensor readings(Hall + IMU) without libraries
 */

#include "Arduino.h"
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define dutyCycle 230 // 15 to 240

//MotorRight ports
#define enableRight 51
#define pwmRight 3
#define avSpeedRight A1

//MotorLeft ports
#define enableLeft 50
#define pwmLeft 2
#define avSpeedLeft A3

//Variables
byte duty = dutyCycle; 
float readAvSpeedRight;
float readAvSpeedLeft;
int count = 0;
unsigned long startTime;
unsigned long dT;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float sensScaleFactor = 131.00; //[LSB/°/s] gain at +- 250 [°/s] configuration DEFAULT ONE
float noiseMean = 0.86; // mean noise on gyroscope gz lecture, averaged over 1000 data points 
float scaledGz;
float aSensScaleFactor = 16384; // [LSB/g] gain at ±2 g configuration DEFAULT ONE
float accNoiseMean = -0.04; // mean noise on accelerometre ax lecture, averaged over 1000 data points 
float scaledAx;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(avSpeedRight, INPUT); //analog input -> average speed motorRight
  pinMode(avSpeedLeft, INPUT); //analog input -> average speed motorLeft
  TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
  pinMode(pwmRight, OUTPUT); //PWM pin motorRight
  pinMode(enableRight, OUTPUT); //enable pin motorRight
  pinMode(pwmLeft, OUTPUT); //PWM pin motorLeft
  pinMode(enableLeft, OUTPUT); //enable pin motorLeft
  digitalWrite(enableRight, HIGH);                
  digitalWrite(enableLeft, HIGH);
  Serial.begin(38400); 
  while (!Serial) continue;
  accelgyro.initialize();
  while(!accelgyro.testConnection()) continue;
  startTime = millis();

}

void loop() {

  // if the pi communicates to stop
   if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "stop"){
      digitalWrite(enableRight, LOW); //disable motorRight                 
      digitalWrite(enableLeft, LOW); //disable motorLeft
    }
    /*  TO TEST
    if (data == "start"){
      digitalWrite(enableRight, HIGH); //enable motorRight                 
      digitalWrite(enableLeft, HIGH); //enable motorLeft
    }
    */
   }
   
  /* TO  TEST
  analogWrite(pwmRight, 255-duty); //dutyCicle to determine between 0 and 255 
  analogWrite(pwmLeft, duty);
  */
  
  if (count < 500){
    if (millis()>startTime + 19){ // 20ms as dT
      analogWrite(pwmRight, 255-duty); //dutyCicle to determine between 0 and 255 
      analogWrite(pwmLeft, duty);
      readAvSpeedRight = analogRead(avSpeedRight);
      readAvSpeedLeft = analogRead(avSpeedLeft);
      dT = millis() - startTime;
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      scaledGz = gz/sensScaleFactor - noiseMean;
      scaledAx = ax/aSensScaleFactor - accNoiseMean;

      //Serial.println(millis()-startTime);
      startTime = millis();
      StaticJsonDocument<120> doc;
      doc["gyro"] = scaledGz;
      doc["accel"] = scaledAx;
      doc["dT"] = dT;
      JsonArray motorSpeed = doc.createNestedArray("motorSpeed");
      motorSpeed.add(readAvSpeedRight);
      motorSpeed.add(readAvSpeedLeft);
      // Send the JSON document over the serial port
      serializeJson(doc, Serial);
      Serial.println();
      count++;
    }
  }
  else{
    digitalWrite(enableRight, LOW); //disable motorRight
    digitalWrite(enableLeft, LOW); //disable motorLeft
  }

}
