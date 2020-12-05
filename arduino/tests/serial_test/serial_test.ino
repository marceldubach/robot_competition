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
      String somestring = Serial.readString();
      //Serial.println("received: ");
      //Serial.println(somestring);
      deserializeJson(jsonBuffer, somestring);

      JsonObject obj = jsonBuffer.as<JsonObject>();

      double left = obj[String("command")][0];
      double right = obj[String("command")][1];
      
      //Serial.println(left);
      //Serial.println(right);
      StaticJsonDocument<200> doc;

      doc["sensor"] = "odometry";
    
      JsonArray command = doc.createNestedArray("cmd");
      command.add(left);
      command.add(right);

      JsonArray vel = doc.createNestedArray("vel");

      // read velocities here
      int vel_left = 100;
      int vel_right = 200;
      vel.add(vel_left);
      vel.add(vel_right);
    
      serializeJson(doc, Serial);
      Serial.println();
  }
  //serializeJsonPretty(doc, Serial);

  delay(1);
}
