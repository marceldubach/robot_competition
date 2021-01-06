#include "Arduino.h"
#include "VelCtrl.h"

void set_Commands(bool enableMotors, int cmdRight, int cmdLeft, byte pwmRight, byte pwmLeft, byte enableRight, byte enableLeft){
    if (enableMotors){
    digitalWrite(enableRight, HIGH);                
    digitalWrite(enableLeft, HIGH);
    // saturation
    if (cmdRight>240){
      cmdRight = 240;
    } else if (cmdRight<15) {
      cmdRight = 15;
    }
    if (cmdLeft>240){
      cmdLeft = 240;
    }else if(cmdLeft<15){
      cmdLeft = 15;
    }
    // set motor speed (fixed speed)
    analogWrite(pwmRight, 255-cmdRight);
    analogWrite(pwmLeft, cmdLeft);
  } else {
    digitalWrite(enableRight, LOW);                
    digitalWrite(enableLeft, LOW);
  };
}

void calculate_Commands(int& cmdLeft,int& cmdRight, double x, double y, double theta, double ref_x, double ref_y, double max_dist[]){
  // assign correct values to ctrlLeft, ctrlRight, based on the waypoint

  // calculate desired heading in [0,2*PI]
  double heading_ref = atan2((ref_y-y), (ref_x-x));
  if (heading_ref<0){ // rescale heading reference in [0,2*PI]
    heading_ref += 2*PI;
  }

  // calculate distance to waypoint
  double dist = sqrt(pow((ref_y-y),2)+pow((ref_x-x),2));

  // if the heading is ok (difference < del_theta), then go straight
  double del_theta = 0.3; // tolerance for angle: 0.3 rad = 17.8°
  if ((fabs(heading_ref-theta)<del_theta) || (fabs(heading_ref-theta)>(2*PI-del_theta))){
    // heading is good -> fast forward
    double forward_dist = 70;
    max_dist[0] = 30;
    max_dist[1] = 30;
    max_dist[2] = 30;
    max_dist[3] = forward_dist;
    max_dist[4] = forward_dist;
    max_dist[5] = forward_dist;
    max_dist[6] = 30;
    max_dist[7] = 30;
    if (dist>1){
      cmdLeft = 220;
      cmdRight = 220;
    }else{
      cmdLeft = 150+dist*50; // give some value between 150 and 200
      cmdRight = 150+dist*50;
    }
  }else{
    double turn_dist = 30; // 50 too high, blocks all the time
    max_dist[0] = turn_dist;
    max_dist[1] = turn_dist;
    max_dist[2] = turn_dist;
    max_dist[3] = turn_dist;
    max_dist[4] = turn_dist;
    max_dist[5] = turn_dist;
    max_dist[6] = turn_dist;
    max_dist[7] = turn_dist;
    // us_sensors at 45° are set to 30
    
    bool turnLeft; // get rotation sense
    if ((heading_ref-theta)>0){
      if (heading_ref-theta<PI){
        turnLeft = true; // turn left
      } else {
        turnLeft = false; // turn right
      }
    } else { // theta > heading_ref
      if ((theta - heading_ref)<PI){
        turnLeft = false;
      } else {
        turnLeft = true;
      }
    }
    if ((fabs(heading_ref-theta)<2*del_theta)||(fabs(heading_ref-theta)>2*PI-2*del_theta)){
      // turn slowly
      if (turnLeft){
        cmdRight = 135;
        cmdLeft = 121;
      }else{ // turn right
        cmdLeft = 135;
        cmdRight = 121;
      }
    } else if((fabs(heading_ref-theta)<4*del_theta)||(fabs(heading_ref-theta)>2*PI-4*del_theta)){
      if (turnLeft){
        cmdRight = 150;
        cmdLeft = 106;
      }else{ // turn right
        cmdLeft = 150;
        cmdRight = 106;
      }
    } else {
      // turn faster
      if (turnLeft){ 
        cmdRight = 165;
        cmdLeft = 91;
      } else {
        cmdRight = 91;
        cmdLeft = 165;
      }
    }
  } // end else turn
}

void calculate_Commands(int& cmdLeft,int& cmdRight, double x, double y, double theta, double ref_x, double ref_y){
  // assign correct values to ctrlLeft, ctrlRight, based on the waypoint

  // calculate desired heading in [0,2*PI]
  double heading_ref = atan2((ref_y-y), (ref_x-x));
  if (heading_ref<0){ // rescale heading reference in [0,2*PI]
    heading_ref += 2*PI;
  }

  // calculate distance to waypoint
  double dist = sqrt(pow((ref_y-y),2)+pow((ref_x-x),2));

  // if the heading is ok (difference < del_theta), then go straight
  double del_theta = 0.3; // tolerance for angle: 0.3 rad = 17.8°
  if ((fabs(heading_ref-theta)<del_theta) || (fabs(heading_ref-theta)>(2*PI-del_theta))){
    // heading is good -> fast forward
    if (dist>1){
      cmdLeft = 220;
      cmdRight = 220;
    }else{
      cmdLeft = 150+dist*50; // give some value between 150 and 200
      cmdRight = 150+dist*50;
    }
  }else{
    bool turnLeft; // get rotation sense
    if ((heading_ref-theta)>0){
      if (heading_ref-theta<PI){
        turnLeft = true; // turn left
      } else {
        turnLeft = false; // turn right
      }
    } else { // theta > heading_ref
      if ((theta - heading_ref)<PI){
        turnLeft = false;
      } else {
        turnLeft = true;
      }
    }
    if ((fabs(heading_ref-theta)<2*del_theta)||(fabs(heading_ref-theta)>2*PI-2*del_theta)){
      // turn slowly
      if (turnLeft){
        cmdRight = 135;
        cmdLeft = 121;
      }else{ // turn right
        cmdLeft = 135;
        cmdRight = 121;
      }
    } else if((fabs(heading_ref-theta)<4*del_theta)||(fabs(heading_ref-theta)>2*PI-4*del_theta)){
      if (turnLeft){
        cmdRight = 150;
        cmdLeft = 106;
      }else{ // turn right
        cmdLeft = 150;
        cmdRight = 106;
      }
    } else {
      // turn faster
      if (turnLeft){ 
        cmdRight = 165;
        cmdLeft = 91;
      } else {
        cmdRight = 91;
        cmdLeft = 165;
      }
    }
  } // end else turn
}
