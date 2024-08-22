#include <ros.h>
#include <arena_control/control_lights.h>

// OBJECT NODE
ros::NodeHandle nh;

// ARDUINO PINES
const int light_pins[2] = {2, 3};


bool callback(arena_control::control_lights::Request & req, arena_control::control_lights::Response & res) {
  nh.loginfo("-------------------")
  digitalWrite(light_pins[0], req.light_states[0] ? HIGH : LOW);
  digitalWrite(light_pins[1], req.light_states[1] ? HIGH : LOW);
  return true;
}


ros::ServiceServer<arena_control::control_lights::Request, arena_control::control_lights::Response> service("lights_control", &callback);

void setup() {
  nh.initNode();
  nh.advertiseService(service);
}

void loop() {
  nh.spinOnce();
  delay(1000);
}