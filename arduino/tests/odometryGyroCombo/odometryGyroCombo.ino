#include "Arduino.h"
#include <ArduinoJson.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"


#define dutyCycle 230 // 15 to 240

//Motor1 ports
#define enable1 51
#define pwm1 3
#define speed1 A0 
#define avSpeed1 A1

//Motor2 ports
#define enable2 50
#define pwm2 2
#define speed2 A2
#define avSpeed2 A3

//Variables
byte duty = dutyCycle; 
float readSpeed;
float readAvSpeed;
float readSpeed2;
float readAvSpeed2;
int count = 0;
unsigned long startTime;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float sensScaleFactor = 131.00; //gain at +- 2g configuration DEFAULT ONE
float noiseMean = 0.86; // mean noise on gyroscope gz lecture, averaged over 1000 data points 

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(speed1, INPUT); //analog input -> actual speed motor1
  pinMode(avSpeed1, INPUT); //analog input -> average speed motor1
  pinMode(speed2, INPUT); //analog input -> actual speed motor1
  pinMode(avSpeed2, INPUT); //analog input -> average speed motor1
  TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
  pinMode(pwm1, OUTPUT); //PWM pin motor1
  pinMode(enable1, OUTPUT); //enable pin motor1
  pinMode(pwm2, OUTPUT); //PWM pin motor2
  pinMode(enable2, OUTPUT); //enable pin motor2
  digitalWrite(enable1, HIGH); //enable motor1                  //CHANGE
  digitalWrite(enable2, HIGH); //enable motor2
  Serial.begin(38400); // // Serial Communication is starting with 9600 of baudrate speed
  while (!Serial) continue;
  accelgyro.initialize();
  startTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  if (count < 500){
    if (millis()>startTime + 19){ // 20ms as DT
      analogWrite(pwm1, 255-duty); //dutyCicle to determine between 0 and 255 
      analogWrite(pwm2, duty);
      readAvSpeed = analogRead(avSpeed1);
      readAvSpeed2 = analogRead(avSpeed2);
      accelgyro.getRotation(&gx, &gy, &gz);
      float scaledGz = gz/sensScaleFactor - noiseMean;

      //Serial.println(millis()-startTime);
      startTime = millis();
      StaticJsonDocument<80> doc;
      doc["gyro"] = scaledGz;
      JsonArray motorSpeed = doc.createNestedArray("motorSpeed");
      motorSpeed.add(readAvSpeed);
      motorSpeed.add(readAvSpeed2);
      // Send the JSON document over the serial port
      serializeJson(doc, Serial);
      Serial.println();
      count++;
    }
  }
  else{
    digitalWrite(enable1, LOW); //enable motor1
    digitalWrite(enable2, LOW); //enable motor2
  }

}
