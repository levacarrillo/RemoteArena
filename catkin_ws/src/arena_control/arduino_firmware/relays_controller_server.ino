#include <ros.h>
#include <arena_control/control_lights.h>

// OBJECT NODE
ros::NodeHandle nh;

// ARDUINO'S PINES
const int light_pins[2]   = {2, 3};
const int charger_pins[2] = {4, 5};

bool light_callback(arena_control::control_lights::Request &req, arena_control::control_lights::Response &res) {
  nh.loginfo("-------------------");
  char buffer[10];
  for(int i=0; i<sizeof(light_pins); i++) {
    if((int)req.light_states[i] == 1) {
      digitalWrite(light_pins[i], LOW);
      sprintf(buffer, "Light %d turned on", i);
    } else {
      digitalWrite(light_pins[i], HIGH);
      sprintf(buffer, "Light %d turned off", i);
    }
    nh.loginfo(buffer);
  }
  return true;
}


ros::ServiceServer<arena_control::control_lights::Request, arena_control::control_lights::Response> light_server("lights_control", &light_callback);

void setup() {
  nh.initNode();
  nh.advertiseService(light_server);
  
  // SETTING PINS MODE
  for(int i=0; i<sizeof(light_pins); i++) {
    pinMode(light_pins[i], OUTPUT);
    digitalWrite(light_pins[i], HIGH);
  }
  for(int i=0; i<sizeof(charger_pins); i++) {
    pinMode(charger_pins[i], OUTPUT);
    digitalWrite(charger_pins[i], HIGH);
  }
}

void loop() {
  nh.spinOnce();
  delay(1000);
}