#include <ros.h>
#include <arena_control/LightBulbsControl.h>
#include <arena_control/RobotChargersControl.h>

// OBJECT NODE
ros::NodeHandle nh;

// ARDUINO'S PINES
int light_pins[]   = {2, 3};
int charger_pins[] = {4, 5};

bool lights_callback(arena_control::LightBulbsControl::Request &req, arena_control::LightBulbsControl::Response &res) {
  nh.loginfo("------------------------");
  char buffer[25];
  for(int i=0; i<(sizeof(light_pins) / sizeof(light_pins[0])); i++) {
    if((int)req.bulbs_state[i] == 1) {
      digitalWrite(light_pins[i], LOW);
      sprintf(buffer, "Light bulb %d turned on", i+1);
    } else {
      digitalWrite(light_pins[i], HIGH);
      sprintf(buffer, "Light bulb %d turned off", i+1);
    }
    nh.loginfo(buffer);
  }
  res.success = true;
  return res.success;
}

bool chargers_callback(arena_control::RobotChargersControl::Request &req, arena_control::RobotChargersControl::Response &res) {
  nh.loginfo("------------------------");
  char buffer[25];
  for(int i=0; i<(sizeof(charger_pins) / sizeof(charger_pins[0])); i++) {
    if((int)req.chargers_state[i] == 0) {
      digitalWrite(charger_pins[i], LOW);
      sprintf(buffer, "Charger %d turned off", i+1);
    } else {
      digitalWrite(charger_pins[i], HIGH);
      sprintf(buffer, "Charger %d turned on", i+1);
    }
    nh.loginfo(buffer);
  }
  res.success = true;
  return res.success;
}

ros::ServiceServer<arena_control::LightBulbsControl::Request, arena_control::LightBulbsControl::Response> lights_server("light_bulbs_state", &lights_callback);
ros::ServiceServer<arena_control::RobotChargersControl::Request, arena_control::RobotChargersControl::Response> chargers_server("chargers_state", &chargers_callback);

// SETTING PINS MODE
void pins_setup() {
  for(int i=0; i<(sizeof(light_pins) / sizeof(light_pins[0])); i++) {
    pinMode(light_pins[i], OUTPUT);
    digitalWrite(light_pins[i], HIGH);
  }
  for(int i=0; i<(sizeof(charger_pins) / sizeof(charger_pins[0])); i++) {
    pinMode(charger_pins[i], OUTPUT);
    digitalWrite(charger_pins[i], HIGH);
  }
}

// RESETTING CHARGERS
void reset_chargers() {
  // TURNING OFF CHARGERS
  for(int i=0; i<(sizeof(charger_pins) / sizeof(charger_pins[0])); i++) {
    digitalWrite(charger_pins[i], LOW);
  }
  // TURNING ON CHARGERS AGAIN
  for(int i=0; i<(sizeof(charger_pins) / sizeof(charger_pins[0])); i++) {
    digitalWrite(charger_pins[i], HIGH);
  }
}

void setup() {
  pins_setup();
  nh.initNode();
  nh.advertiseService(lights_server);
  nh.advertiseService(chargers_server);
  reset_chargers();
}

void loop() {
  nh.spinOnce();
  delay(1000);
}
