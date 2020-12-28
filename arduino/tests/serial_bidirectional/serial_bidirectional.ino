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
  Serial.begin(38400);
  state = 0;
  while(!Serial) continue;
  delay(10000);
  Serial.println("ready");
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
      String x_string = String(x,1);
      String y_string = String(y,1);
      String theta_string = String(theta,2);
      position.add(x_string);
      position.add(y_string);
      position.add(theta_string);
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
    if (x<8){
      x+=0.001;
    } else {
      x-=0.001;
    }
    if (y<8){
      y+=0.001;
    } else {
      y-=0.001;
    }
    if (theta<6.28){
      theta += 0.001;
    } else {
      theta -= 0.001;
    }    
  }
  
  //delay(10);
}
