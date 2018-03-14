# Smart Car Basics

This project is for emulating a self-driving car in the [Pheeno robot](http://www.math.ucla.edu/~bertozzi/papers/PheenoICRA2016.pdf) 

## Software Dependencies

- Python 2.7
- openCV 2,4
- Arduino

## Outline of the project

The project is divided in to two levels, the top level is the raspberry-pi and the bottom level is the teensy micocontroller. The arduino programs are developed in C++ and is used to program the teensy microcontroller and the raspberry-pi programs are developed in python language.
The arduino folder contains the arduino programs and the raspberrypi folder contains the python programs.

## How to run the project

1. Upload the arduino code to the teensy  microcontroller
2. add the smart_car_1.4.py to the bash.rc of the raspberry
3. The robot should emulate a self driving car when restarted
