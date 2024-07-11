#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

#define BAUD 200000 

ros::NodeHandle nh;

int led[6] = {2, 3, 4, 5, 6, 7};

void disconnect_chargers()
{
  
  digitalWrite(4, LOW); 
  digitalWrite(5, LOW); 
  delay(1000);
  digitalWrite(4, HIGH); 
  digitalWrite(5, HIGH); 
}

void lightsCallback(const std_msgs::Int8MultiArray& msg) {

  nh.loginfo("---------------------");
  if(msg.data[0]) {
    digitalWrite(led[0], LOW);
    nh.loginfo("Light 1 turned ON");
  }
  else {
    digitalWrite(led[0], HIGH);
    nh.loginfo("Light 1 turned OFF");
  }

  if(msg.data[1]) {
    digitalWrite(led[1], LOW);
    nh.loginfo("Light 2 turned ON");
  }
  else {
    digitalWrite(led[1], HIGH);
    nh.loginfo("Light 2 turned OFF");
  }
}

void chargersCallback(const std_msgs::Int8MultiArray& msg) {
  
  if(msg.data[0]) digitalWrite(4, LOW);
  else digitalWrite(4, HIGH);

  if(msg.data[1]) digitalWrite(5, LOW);
  else digitalWrite(5, HIGH);
}

ros::Subscriber<std_msgs::Int8MultiArray> subLights("/turn_lights", &lightsCallback);
ros::Subscriber<std_msgs::Int8MultiArray> subChargers("/chargers_relays", &chargersCallback);

void setup() {

  //Setting and turning off all relays
  for(int i=0; i<6; i++) {
     pinMode(led[i], OUTPUT);
     digitalWrite(led[i], HIGH); 
  }

  nh.initNode();
  nh.subscribe(subLights);
  nh.subscribe(subChargers);
  disconnect_chargers();
}

void loop() {
  
  nh.spinOnce();
  delay(100);
}
