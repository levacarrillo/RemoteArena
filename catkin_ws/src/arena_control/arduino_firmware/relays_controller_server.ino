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
  // TURNING ON LIGHTS
  for (int i=0; i<sizeof(light_pin); i++){
    digitalWrite(light_pin[i], LOW);
  }
  delay(600);
  // TURNING OFF LIGHTS AGAIN
  for (int i=0; i<sizeof(light_pin); i++){
    digitalWrite(light_pin[i], HIGH);
  }
}

// RESETTING CHARGERS
void reset_chargers() {
  // TURNING OFF CHARGERS
  for (int i=0; i<sizeof(charger_pin); i++){
    digitalWrite(charger_pin[i], LOW);
  }
  delay(600);
  // TURNING ON CHARGERS AGAIN
  for (int i=0; i<sizeof(charger_pin); i++){
    digitalWrite(charger_pin[i], HIGH);
  }
}

bool getParams() {
  bool params_loaded = true;
  if(!nh.getParam("/hardware/light_bulbs", light_bulb_status, 2)) { params_loaded = false; }
  if(!nh.getParam("/hardware/chargers", charger_status, 2)) { params_loaded = false; }

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
      digitalWrite(light_pin[i], !light_bulb_status[i]);
      digitalWrite(charger_pin[i], charger_status[i]);
    }
  }

  nh.spinOnce();
  delay(50);
}
