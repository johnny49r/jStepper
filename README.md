# jStepper 

Arduino high performance stepper library for ATMega boards. 

The library is targeted for the Arduino ATMega board family and can be adapted
to many other embedded systems. 
There are certainly simpler ways to control stepper motors such as the Arduino 
'stepper' library or the popular AccelStepper library. 
This library was written for a CNC control application that required high 
performance and unique features that weren't available in other open-source projects. 

### FEATURES

1) Supports up to 3 stepper motors per instance of the library.
2) Multiple instances limited to the number of available 16 bit timers.
3) High speed stepping up to 20,000 PPS on three concurrent motors.
4) Very slow step rates down to one step every 72 minutes.
5) Accurate step pulse timing with minimal timebase jitter.
6) Constant speed mode (square profile).
7) Real time linear acceleration (trapeziod & triangle profiles).
8) Syncronize motors to arrive at the destination at the same time.
9) Plan-ahead feature eliminates overhead between motor movements.
10) Primitive functions for single stepping & endstop detection (homing).
11) Built-in homing function, interrupt driven (non-blocking).
12) Interrupt driven stepping and planning operates in the background (multi-tasking).
13) Library manages I/O and interrupt redirection dynamically for multiple instances.
14) Supports relative and absolute positioning.
15) User programmed timer interrupt callback.
16) Fast I/O and math functions available to your program. See the jsio.h & jsmath.h files.

### HARDWARE

The general hardware model is an Arduino (or similar) ATMega board with a shield 
containing stepper driver modules. This is very similar to the popular RAMPS architecture.
This library will also work with hybrids such as the Rambo and others.

The code generates STEP and DIRECTION signals in real time. 
The user supplies control pin information via a template structure.

The library has been tested on the ATMega 2560 platform. It should also 
work on the ATMega 328 and other derivatives of Atmel CPU's.

### EXAMPLES

Please see the examples folder.

### INSTALLATION

Download from Github as a ZIP file.
Place the ZIP file in your Arduino/libraries directory.
In the Arduino IDE, select Sketch->Include Library->Add .ZIP Library to install.
Your sketch should now have "#include <jStepper.h>". If not add it manually.

### USAGE

You will need to initialize a structure (jsMotorConfig) and pass it to the library
in the begin() function. Each instance of the library will need a different copy 
of the jsMotorConfig with different settings.
Each copy of the jsMotorConfig will specify the timer it will use; 1, 3, 4, or 5. 
Once the structure has been passed to the library it is ready to accept commands.

NOTE: The library pre-allocates interrupt service routines for all available 16 bit
timers. This means that you can't use the ISR(TIMERn_CMPn_vect) or it will conflict
with the library. However you can add your own interrupt handler through the 
'addTimerCallBack' function (see examples).

### TODO

- [ ] Documentation
- [ ] Examples
- [ ] Installation

### DISCLAIMER 

This code is functional and all features work but it is a work-in-progress. I 
welcome your suggestions for improvement.



