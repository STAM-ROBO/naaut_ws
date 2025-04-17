#!/bin/bash
sudo ./bin/arduino-cli compile --fqbn arduino:megaavr:nona4809 motor-driver/motor-driver.ino 
sudo ./bin/arduino-cli upload -p /dev/ttyACM0 --verbose --fqbn arduino:megaavr:nona4809 motor-driver/motor-driver.ino 
