/* this script implements a bidirectional communication with RaspberryPi
 * Load it and launch raspi/tests/JsonSerial/bidirectional_com.py
 * 
 * State initialized to 0 -> no prints on Serial
 * The program reads the prints on Serial from Python, adapts accordingly
 * and replies to python
 */

#include <ArduinoJson.h>

StaticJsonDocument<200> receive_msg;

double x = 1;
double y = 1;
double theta = 0;

double cmdLeft;
double cmdRight;

int state = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  state = 0;
  while(!Serial) continue;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    String line = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(receive_msg, line);
    if (error){
      Serial.println("Deserialize failed");
    } else {
      int s = receive_msg["state"];
      double cLeft = receive_msg["cmd"][0];
      double cRight = receive_msg["cmd"][1];
      state = s;
      cmdLeft = cLeft;
      cmdRight= cRight;

      const int capacity=200;
      StaticJsonDocument<capacity>send_msg;
  
      send_msg["state"] = state;
      JsonArray position = send_msg.createNestedArray("pos");
      position.add(x);
      position.add(y);
      position.add(theta);
      JsonArray command = send_msg.createNestedArray("cmd");
      command.add(cmdLeft);
      command.add(cmdRight);
      serializeJson(send_msg, Serial);   
    }/*

    if (state==1){
      const int capacity=200;
      StaticJsonDocument<capacity>send_msg;
  
      send_msg["state"] = state;
      JsonArray position = send_msg.createNestedArray("pos");
      position.add(x);
      position.add(y);
      position.add(theta);
      JsonArray command = send_msg.createNestedArray("cmd");
      command.add(cmdLeft);
      command.add(cmdRight);
      serializeJson(send_msg, Serial);
    }
    */
    
  }

  // simulation: position changes over time...
  if (state == 1){
    x += 0.01;
    y += 0.02;
    theta += 0.01;
  } 
  delay(10);
}
