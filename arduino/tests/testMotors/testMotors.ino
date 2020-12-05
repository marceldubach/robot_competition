#include "Arduino.h"
#include <ArduinoJson.h>

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
float readSpeed;
float readAvSpeed;
float readSpeed2;
float readAvSpeed2;
float x = 0;
boolean b = false;
int count = 0;


void setup() {
  // put your setup code here, to run once:
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
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  while (!Serial) continue;
}

void loop() {

  /*  
  readSpeed = analogRead(speed1);
  Serial.println("Motor 1 speed: ");
  Serial.println(readSpeed);
  readAvSpeed = analogRead(avSpeed1);
  Serial.println("Motor 1 average speed: ");
  Serial.println(readAvSpeed);
  readSpeed2 = analogRead(speed2);
  //Serial.println("Motor 2 speed: ");
  //Serial.println(readSpeed2);
  readAvSpeed2 = analogRead(avSpeed2);
  //Serial.println("Motor 2 average speed: ");
  //Serial.println(readAvSpeed2);
  */  
  if (count < 50){
    analogWrite(pwm1, 255-duty); //dutyCicle to determine between 0 and 255 
    analogWrite(pwm2, duty);
    readAvSpeed = analogRead(avSpeed1);
    readAvSpeed2 = analogRead(avSpeed2);
    
    
    StaticJsonDocument<60> doc;
    JsonArray motorSpeed = doc.createNestedArray("motorSpeed");
    motorSpeed.add(readAvSpeed);
    motorSpeed.add(readAvSpeed2);
    // Send the JSON document over the serial port
    serializeJson(doc, Serial);
    Serial.println();
  

    delay(100);
    count++;
  }
  else{
    digitalWrite(enable1, LOW); //enable motor1
    digitalWrite(enable2, LOW); //enable motor2
  }
}
