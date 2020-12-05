#include <ArduinoJson.h>


/*
#define enable1 51
#define pwm1 3

#define enable2 50
#define pwm2 2
*/

StaticJsonDocument<200> jsonBuffer;

void setup() {
  // put your setup code here, to run once:

  //pinMode(pwm1, OUTPUT); //PWM pin motor1
  //pinMode(enable1, OUTPUT); //enable pin motor1
  //pinMode(pwm2, OUTPUT); //PWM pin motor2
  //pinMode(enable2, OUTPUT); //enable pin motor2

  //digitalWrite(enable1, LOW ); //enable motor1
  //digitalWrite(enable2, LOW); //enable motor2
  
  

  Serial.begin(9600);
  while(!Serial) continue;


}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
      int somestring = Serial.read();
      StaticJsonDocument<200> doc;

      doc["sensor"] = "odometry";
      doc["time"] = 333;
    
      JsonArray data = doc.createNestedArray("data");
      data.add(48.2342);
      data.add(243.434);
    
      serializeJson(doc, Serial);
  }
  //serializeJsonPretty(doc, Serial);

  delay(1);
}
