#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_srv/Empty.h>

// SET THE RATE COMUNICATION
#define BAUD 200000 

// OBJECT NODE
ros::NodeHandle nh;

// ARDUINO'S PINES
int pin[2] = {2, 3};

bool controlRelays(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  
  nh.loginfo("----------------------");
  
  digitalWrite(pin[0], HIGH);
  digitalWrite(pin[1], HIGH);
 
  return true;
}

void setup() {
  nh.initNode();

  // SETTING AND TURNNING OFF ALL RELAYS
  for(int i=0; i<6; i++) {
     pinMode(pin[i], OUTPUT);
     digitalWrite(pin[i], HIGH); 
  }
  
  // STARTING SERVICE
  nh.advertiseService("control_relays", controlRelays);
}

void loop() {
  
  nh.spinOnce();
  delay(100);
}
