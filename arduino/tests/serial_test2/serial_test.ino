#include <ArduinoJson.h>

StaticJsonDocument<200> jsonBuffer;

double omega;
double acc;

String stringReceived;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) continue;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    stringReceived = Serial.readString();

    //deserializeJson(jsonBuffer, stringReceived);
    Serial.println(stringReceived);
    //JsonObject obj = jsonBuffer.as<JsonObject>();

    //double left  = obj[String("command")][0];
    //double right = obj[String("command")][1];
  }
  
  StaticJsonDocument<200> doc;

  JsonArray motor = doc.createNestedArray("mot");
  int motorLeft = 100;
  int motorRight = 200;
  motor.add(motorLeft);
  motor.add(motorRight);

  JsonArray gyro = doc.createNestedArray("gyro");
  double omega = 3.1415*2/10;
  double acc = 0;
  gyro.add(omega);
  gyro.add(acc);

  serializeJson(doc, Serial);
  Serial.println();

  delay(50);

}
