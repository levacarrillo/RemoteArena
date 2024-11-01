#include <ros.h>

ros::NodeHandle nh;

// SETTING UP ARDUINO'S PINS
int light_pin[2] = {2, 3};
int charger_pin[2] = {4, 5};

// PARAMS VARIABLES
bool charger_status[2] = {true, true};
bool light_bulb_status[2]= {false, false};

// RESETTING LIGHTS
void reset_lights() {
  for (int i=0; i<sizeof(light_pin); i++){
    // TURNING ON LIGHTS
    digitalWrite(light_pin[i], HIGH);
    delay(800);
    // TURNING OFF LIGHTS AGAIN
    digitalWrite(light_pin[i], LOW);
  }
}

// RESETTING CHARGERS
void reset_chargers() {
  for (int i=0; i<sizeof(charger_pin); i++){
    // TURNING OFF CHARGERS
    digitalWrite(charger_pin[i], LOW);
    delay(800);
    // TURNING ON CHARGERS AGAIN
    digitalWrite(charger_pin[i], HIGH);
  }
}

bool getParams() {
  bool params_loaded = true;
  if(!nh.getParam("/arena_hardware/light_bulbs", light_bulb_status, 2)) { params_loaded = false; }
  if(!nh.getParam("/arena_hardware/chargers", charger_status, 2)) { params_loaded = false; }

  return params_loaded;
}

void setup() {
  nh.initNode();

  for (int i=0; i<2; i++) {
    pinMode(light_pin[i],  OUTPUT);
    pinMode(charger_pin[i], OUTPUT);
  }

  reset_lights();
  reset_chargers();
}

void loop() {
  if(!getParams()) {
    nh.logwarn("-----------------------------------------------");
    nh.logwarn("Arena's control params not loaded correctly");
  } else {
    for(int i=0; i<2; i++) {
      digitalWrite(light_pin[i], light_bulb_status[i]);
      digitalWrite(charger_pin[i], light_bulb_status[i]);
    }
  }

  nh.spinOnce();
  delay(50);
}
