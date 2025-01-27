# ESP_PID_OMNIROVER
A simple implementation of a PID controller for a rover with omnidirectional mecanum wheels.

This is still in its earliest stage, the final product should have a GUI that allows anyone to use the rover easily and also have a 
framework that allows computer vision to do further tests with a machine learning algorithm so that the one or more rovers can be 
automated to do specific tasks.

At the moment it consist in a PID controller written in C using the ESP-IDF framework since the microcontroller used is a ESP32-S3 
and a Matlab code that allows any user to input the velocity it wants on the “Y” and “X” axis and the rotation that the rover will have. 

## IDF Component Manager Manifest File
dependencies:
  espressif/led_strip: "^3.0.0"
  ## Required IDF version
  idf:
    version: ">=4.1.0"

The implementetion of the PID controller:

Implementetion of remote control:
  
  
