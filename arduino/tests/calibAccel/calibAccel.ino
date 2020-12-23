// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#include <ArduinoJson.h>
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
//float sensScaleFactor = 131.00;
//float noiseMean = 0.86;

unsigned long startTime;

#define dutyCycle 230 // 0 to 255
#define PI 3.1415926535897932384626433832795

//Motor1 ports
#define enable1 51
#define pwm1 3
#define speed1 A0 //May need to filter the signal
#define avSpeed1 A1

//Motor2 ports
#define enable2 50
#define pwm2 2
#define speed2 A2
#define avSpeed2 A3

//Variables
byte duty = dutyCycle; 
int count = 0;
float sumAx = 0;
float accNoiseMean;
float Temp_accNoiseMean = -0.04;
float aSensScaleFactor = 16384; // [LSB/g] gain at ±2 configuration DEFAULT ONE


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
    pinMode(pwm1, OUTPUT); //PWM pin motor1
    pinMode(enable1, OUTPUT); //enable pin motor1
    pinMode(pwm2, OUTPUT); //PWM pin motor2
    pinMode(enable2, OUTPUT); //enable pin motor2
    digitalWrite(enable1, LOW); //enable motor1                  
    digitalWrite(enable2, LOW); //enable motor2
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    startTime = millis();

}

void loop() {
  if (count < 1000){

      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      sumAx += ax/aSensScaleFactor - Temp_accNoiseMean;
      StaticJsonDocument<60> doc;
      doc["raw accel"] = ax;
      doc["scaled accel"] = ax/aSensScaleFactor;
      // Send the JSON document over the serial port
      serializeJson(doc, Serial);
      Serial.println();
      startTime = millis();
      count++;
 
  }
  else{
    accNoiseMean = float(sumAx/count);
    Serial.println(accNoiseMean);
  }
    
}
