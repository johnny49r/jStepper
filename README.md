# jStepper
Arduino high performance stepper library for ATMega boards.

## FEATURES:
1) Supports up to 3 stepper motors per instance of the library.
2) Multiple instances limited to the number of available 16 bit timers.
3) High speed stepping up to 20,000 PPS on three concurrent motors.
4) Accurate step pulse timing with minimal timebase jitter.
5) Constant speed mode (square profile).
5) Real time linear acceleration (trapeziod & triangle profiles).
6) Syncronize motors to arrive at the destination at the same time.
7) Primitive functions for stepping & endstop detection (homing).
8) Interrupt driven step engine doesn't require user program intervention.
9) Non blocking allows multi-tasking.

## HARDWARE

The general hardware model is an Arduino board with a shield containing
stepper driver modules. This is very similar to the popular RAMPS architecture.
This library will also work with hybrids such as the Rambo.

The code generates STEP and DIRECTION signals in real time. 
The user supplies control pin information via a template header file.

The library has been tested on the ATMega 2560 platform. It should also 
work on the ATMega 328 and other derivatives of Atmel CPU's.

## EXAMPLES

Please see the examples folder.

## INSTALLATION

## TODO
- [ ] Documentation
- [ ] Examples
- [ ] Installation

## DISCLAIMER 

This code is functional and all features work but it is a work-in-progress. 
In low performance embedded systems unique solutions are required to meet 
performance goals. Some of my methods are unorthodox but they get the 
job done. 
I welcome suggestions for improvement.

