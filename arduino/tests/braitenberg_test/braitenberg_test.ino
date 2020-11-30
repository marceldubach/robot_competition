#include "UltrasoundSensors.h"

byte trigger[] =  {25, 23, 27, 29, 31, 33, 35};
byte echo[] =     {24, 22, 26, 28, 30, 32, 34};

int n_sensors(7);

UltrasoundSensors us_sensors(trigger, echo, n_sensors);

// UltrasoundSensors us_sensors; // version without constructor
int threshold[] = {0,0,0,0,0,0,0};

#define enable1 51
#define pwm1 3

#define enable2 50
#define pwm2 2

byte motorLeft[] = {50,-50,-50,-50,0,0,50};
byte motorRight[] = {50,50,0,0,-50,-50,-50};

byte commandLeft = 0;
byte commandRight = 0;

void setup() {
  // put your setup code here, to run once:
  // this should be unnecessary with the constructor
  for(int i=0;i<7;i++){
    pinMode(trigger[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }
  //TCCR3B = TCCR3B & B11111000 | B00000001;
  //TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
  
  pinMode(pwm1, OUTPUT); //PWM pin motor1
  pinMode(enable1, OUTPUT); //enable pin motor1
  pinMode(pwm2, OUTPUT); //PWM pin motor2
  pinMode(enable2, OUTPUT); //enable pin motor2

  // turn Hall sensors on
  digitalWrite(enable1, LOW ); //enable motor1
  digitalWrite(enable2, LOW); //enable motor2
  
  Serial.begin(9600);
  
}

void loop() {
  
  // put your main code here, to run repeatedly:
  
  
  double distances[n_sensors] = {0,0,0,0,0,0,0};
  //us_sensors.readUS(trigger, echo, distances, n_sensors); // Doesn't work
  
  // works for up to 3 sensors
  for(int i=0;i<2;i++){
    digitalWrite(trigger[i], LOW);
    //delayMicroseconds(5);
    digitalWrite(trigger[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger[i], LOW);

    long duration = pulseIn(echo[i], HIGH);
    distances[i] = (duration/2)/29.1;
    Serial.println(distances[i]);
  }
  
  //double distances[7] = {20,40,40,40,40,40,40};

  for(int i=0;i<7;i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.println(distances[i]);
  }
  
  for(int i=0;i<7;i++){
    if (distances[i]<100){
      threshold[i]=1;
    } else {
      threshold[i]=0;
    }
  }
  
  commandLeft = 0;
  commandRight = 0;
  for(int i=0;i<7;i++){
    commandLeft += threshold[i]*motorLeft[i];
    commandRight += threshold[i]*motorRight[i];
  }
  
  commandLeft = 128 - commandLeft;
  commandRight = 128 + commandRight;
  
  if (commandLeft>255){
    commandLeft = 255;
  }
  if (commandLeft<=0){
    commandLeft = 0;
  }
  
  if (commandRight>255){
      commandRight = 255;
  }
  if (commandRight <0){
      commandRight = 0;
  }

  
  //analogWrite(pwm1, commandLeft); // dutyCicle in (0,255)  //a digital signal(square wave) as output
  //analogWrite(pwm2, commandRight);

  byte left = 228;
  byte right = 228;
  analogWrite(pwm1, left);
  analogWrite(pwm2, right);
  Serial.print("Command left: ");
  Serial.println(commandLeft);
  Serial.print("Command right: ");
  Serial.println(commandRight);

  /*
  byte duty = 220;
  analogWrite(pwm1, 255-duty);
  analogWrite(pwm2, duty);
  */

  delay(1000);
}
