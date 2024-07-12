# Arena Control Directory

**Description:** This directory contains code to compile and upload firmware to micro-controller Arduino Mega using arduino-mk tools. This firmware controls a relays bank for turning on/off arena's lights and the chargers state.

## Prerrequisites

1. Arduino Mega connected properly via USB port.
2. Dependencies
    1. `arduino-mk`
    2. `screen`

## Build
1. Go into relays_controller directory
2. Run the following commands
```
make

## Upload to micro-controller
make upload clean

```
**IMPORTANT:** Makefile is made for Arduino Mega 2560, if you want to use another Arduino distribution you have to specify it into.
