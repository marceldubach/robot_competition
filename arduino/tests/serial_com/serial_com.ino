#include <ArduinoJson.h>

double x = 0.5;
double y = 0.5;
double theta = 0;

int state = 0;

int cmdRight = 10;
int cmdLeft = 10;


StaticJsonDocument<200> receive_msg;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  while (!Serial) continue;

  Serial.println("ready"); // Arduino setup completed
 
}

void loop() {
if (Serial.available()>0){
    String line = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(receive_msg, line);
    Serial.flush();
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
      //String x_string = String(x,1);
      //String y_string = String(y,1);
      //String theta_string = String(theta,2);
      position.add(x);
      position.add(y);
      position.add(theta);
      JsonArray command = send_msg.createNestedArray("cmd");
      command.add(cmdLeft);
      command.add(cmdRight);
      serializeJson(send_msg, Serial);
      Serial.println();
      
    }
    
  }

  // simulation: position changes over time...
  if (state == 1){
    if (x<8){
      x+=0.01;
    } else {
      x-=0.01;
    }
    if (y<8){
      y+=0.01;
    } else {
      y-=0.01;
    }
    if (theta<6.28){
      theta += 0.01;
    } else {
      theta -= 0.01;
    }    
  }
  
  //delay(10);

}
