/* First test of the gripper
 *  19.11.2020
 */
#include <Servo.h>
#define DFLT 7
enum State_enum {LOWERGRIPPER, CLOSEGRIPPER, RAISEGRIPPER, OPENGRIPPER, OPENBACKDOOR, CLOSEBACKDOOR, CAM};
Servo myHighTorqueServo;  // create servo object to control high torque servo
Servo myMicroServoLeft;   // create servo object to control micro left
Servo myMicroServoRight;  // create servo object to control micro right
Servo myCamServo;         // create servo object to control cam servo
Servo myBackDoorServo;    // create servo object to control back door servo
int state = DFLT;

void setup() {
  myHighTorqueServo.attach(11,400,2550); //400us-2550us DFROBOT high torque 
  myMicroServoLeft.attach(10,900,2100);  // (pin, min, max) // for HC-82 left
  myMicroServoRight.attach(9,900,2100);  // (pin, min, max) // for HC-82 right
  myCamServo.attach(8,750,2250);  // (pin, min, max) // for cam
  myBackDoorServo.attach(7,750,2250);  // (pin, min, max) // for back door
  Serial.begin(9600);
  state = LOWERGRIPPER;
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
      if (data.equals("GO")){
          state = LOWERGRIPPER;
    }
  }
  */
  switch(state)
  {
    case LOWERGRIPPER:
      myHighTorqueServo.write(15);
      delay(1000);
      state = CLOSEGRIPPER;
      delay(100);
      break;

    case CLOSEGRIPPER:
      myMicroServoLeft.write(40);
      myMicroServoRight.write(160);
      delay(1000);
      state = RAISEGRIPPER;
      delay(100);
      break;

    case RAISEGRIPPER:
      myHighTorqueServo.write(160);
      delay(2000);
      state = OPENGRIPPER;
      delay(120);
      break;
      
    case OPENGRIPPER:
      myMicroServoLeft.write(80);
      myMicroServoRight.write(120);
      delay(2000);
      state = OPENBACKDOOR;
      delay(100);
      break;

    case OPENBACKDOOR:
      myBackDoorServo.write(60);
      delay(1000);
      state = CAM;
      delay(100);
      break;

    case CAM:
      for(int i = 0; i < 5; i++){
        myCamServo.write(0);
        delay(500);
        myCamServo.write(70);
        delay(500);
      }
      state = CLOSEBACKDOOR;
      delay(100);
      break;

    case CLOSEBACKDOOR:
      myBackDoorServo.write(165);
      delay(1000);
      state = DFLT;
      Serial.println("end");
      delay(100);
      break;
       
    case DFLT:
      myHighTorqueServo.write(65);
      myMicroServoLeft.write(80);
      myMicroServoRight.write(120);  // 170 closed 
      myBackDoorServo.write(160);
      delay(1000);
      break;
  }
}
