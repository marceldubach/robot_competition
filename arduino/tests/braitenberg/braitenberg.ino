#include <Arduino.h>

// define pins and variables for the ultrasensors (US)
uint8_t trigger[7] =  {25, 23, 27, 29, 31, 33, 35};
uint8_t echo[7] =     {24, 22, 26, 28, 30, 32, 34};

int n_US = 7;
double distances[] = {0, 0, 0, 0, 0, 0, 0}; //initialize distances

// define variables for millis()
unsigned long previousMillis = 0; 
const long interval = 100;  // ms

// define pins and variables for motors
int threshold[] = {0, 0, 0, 0, 0, 0, 0};

#define enable1 51
#define pwm1 3

#define enable2 50
#define pwm2 2

int commandLeft, commandRight, SumDotProd;

// BRAITENBERG MATRIX - TO BE THUNED
int braitenberg[14] = {  600, -100, 300,  -200, -300, -100, 400,
                         600, 400, -100, -300, -200, 300, -100};
/*
int braitenberg[14] = {  40, 15, 10, -15, -25, -30, 25,
                             40, 25, -30, -25, -15, 10, 15};
*/
/*int braitenberg[14] = {0,0,0,-10,0,0,0,
                          0,0,0,-10,0,0,0};*/

void setup() {  // initializing loop 

  Serial.begin(9600);

  for (int i = 0; i < n_US; i++) { // set up Echo and Trigger pins
    pinMode(trigger[i], OUTPUT);
    pinMode(echo[i], INPUT);
  }

  pinMode(pwm1, OUTPUT);        //PWM pin motor1
  pinMode(enable1, OUTPUT);     //enable pin motor1
  pinMode(pwm2, OUTPUT);        //PWM pin motor2
  pinMode(enable2, OUTPUT);     //enable pin motor2

  digitalWrite(enable1, HIGH);  //enable motor1
  digitalWrite(enable2, HIGH);   //enable motor2

  //TCCR3B = TCCR3B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz on pin 5, 3, 2
}

void loop() {
  unsigned long currentMillis = millis(); // retrieve current value of millis
  if (currentMillis - previousMillis >= interval) {

    cli(); //stop interrupts

    previousMillis = currentMillis;

    for (int i = 0; i < n_US; i++) {
      digitalWrite(trigger[i], LOW);
      delayMicroseconds(5);
      digitalWrite(trigger[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger[i], LOW);

      unsigned long duration = pulseIn(echo[i], HIGH);
      distances[i] = (duration / 2) / 29.1; // => potential issue changing variable type
    }

    sei(); // restart interrupts
    
    for (int i = 0; i < n_US; i++) { // print the distances
      //Serial.print(distances[i]);
      //Serial.print("   ");
    }
    //Serial.print("\n");
    
  }// end parenthesis for millis() timed loop
  //Serial.print("Threshold:   ");
  for (int i = 0; i < n_US; i++) {
    if (distances[i] < 50) {
      threshold[i] = 1;
    }
    else {
      threshold[i] = 0;
    }
    
    //Serial.print(threshold[i]);
    //Serial.print("   ");
    
  }
  //Serial.println();
  

  commandLeft = 0;
  commandRight = 0;

  // multiplie sensor input by braitenberg array and get outputs using a threshold on the sensor values
  for (int j = 0; j < 2; j++) {
    SumDotProd = 0; //reset for next row x col product
    for (int i = 0; i < n_US; i++) {
      SumDotProd += threshold[i]/distances[i] * braitenberg[j * n_US + i]; // (j * n_US + i) gives array position
    }
    if (j == 0) {
      commandLeft = SumDotProd;
      Serial.print("CMD left: ");
      Serial.println(commandLeft);
    }
    else if (j == 1) {
      commandRight = SumDotProd;
      Serial.print("CMD right: ");
      Serial.println(commandRight);
    }
    if (j == 1){
    }
    
  }

  // to debug //////////////////////////////////////
  //commandRight = -20;
  //commandLeft = -20;
  //////////////////////////////////////////////////
  
  // center and saturate control input
  Serial.print("L:");
  Serial.print(commandLeft);
  Serial.print(", --- R:");
  Serial.println(commandRight);
  
  commandLeft = 128 + commandLeft;
  commandRight = 128 - commandRight;

  if (commandLeft>240){
    commandLeft = 240;
  }
  if (commandLeft<=10){
    commandLeft = 10;
  }
  
  if (commandRight>240){
      commandRight = 240;
  }
  if (commandRight <10){
      commandRight = 10;
  }
  
  byte cmdLeft = (byte)(commandLeft%255);
  byte cmdRight =(byte)(commandRight%255);
  
  /*
  Serial.print("Command left: ");
  Serial.print(cmdLeft);
  Serial.print("   Command right: ");
  Serial.println(cmdRight);*/

  analogWrite(pwm2, cmdLeft); // dutyCicle in (0,255)  //a digital signal(square wave) as output
  analogWrite(pwm1, cmdRight);

  

  delay(10);
}
