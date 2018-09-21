// ==========================================================================
//
// jStepper.h
// 
// 'jStepper' a stepper motor library for Arduino / AVR hardware 
//  Copyright (C) 2018 by John Hoeppner
//  https://github.com/johnny49r/jStepper
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ==========================================================================
//
//  This library is a mashup of Marlin, Grbl, AccelStepper, and other open 
//  source references. The library attempts to do much of the planning and
//  execution of multiple stepper motors in applications such as 3D printers, 
//  CNC machines, robotics, etc.
//
//  FEATURES:
//  >> Up to three motors supported for each instance of the library. Number
//     of instances is limited by available 16 bit timers (see below).
//  >> High speed stepping up to 20,000 PPS concurently for all motors. This
//	   will be reduced if more than one instance of the library is operating
//     concurently.
//  >> Accurate pulse timing and minimal timebase jitter.
//  >> Real time acceleration / deceleration profiles using the well documented
//     step interval formula _cn = _cn - ((2*_cn)/(4*n)+1)
//  >> Motor async operation with independent speeds & acceleration profiles.
//  >> Motor syncronization calculates movements and adjusts timing of
//     motors to ensure they track and arrive at the same time. 
//  >> Functions for axis homing i.e. single step and endstop detect.
//  >> Multiple instances of the library are supported with restrictions 
//	   on number of 16 bit timers and use by other other functions.
// 
//  This library has been tested on the ATMega 2560 board but should 
//  work on the ATMega 328 as well. 
//
//  Note that each instance of the library uses one of the 16 bit timers.
//  User must allocate the correct timer to avoid conflict with other libraries
//  or PWM functions that employ timers.
//
//  See JSCONFIG.H to set specific user configuration for each instance.
//
// ==========================================================================
//
// RELEASE V-1.0 09/15/2018
//

#ifndef _JSTEPPER_H
#define _JSTEPPER_H

//### Arduino included libs ###
#include <stdlib.h>
#include <Arduino.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

//### local header files
#include "jsmath.h"
#include "jsio.h"
#include "jsconfig.h"


// ==========================================================================
// general configuration 
//
#define MAX_ACCELERATION 100000   	// accel limit
#define MIN_STEP_INTERVAL 50   		// 100 usec min interval
#define MAX_CRUISE_RATE 20000      	// max step speed in PPS
#define MAX_ACCEL_INTERVAL 32767  	// timer overflow limit

// no acceleration needed if step rate slower than this because motor
// is producing max torque at slow speeds. Adjust this according to 
// motor torque specifications. START SPEED is step rate in PPS.
#define MOTOR_START_SPEED 1000

// MAX_SPEED is derived from the motor manufacturers spec
// MAX_SPEED = max rpm (600) / 60 = 10 RPS * mm/rev
// Change these params or override with setMaxSpeed()
#define MAX_MOTOR_RPM 750.0	// motor torque declines above this rpm
#define MOTOR_DISTANCE_PER_REV 8.0	// mm moved in 1 rev of the motor
//#define MAX_SPEED ((MAX_MOTOR_RPM / 60.0) * MOTOR_DISTANCE_PER_REV)
#define MAX_SPEED 200   // 200 mm/sec (20000 pps)

// ==========================================================================
// NUM_MOTORS should always be = 3
//
#define NUM_MOTORS 3

//
// motor nums
//
enum {
	MOTOR_0=0,
	MOTOR_1,
	MOTOR_2,
	MOTOR_ALL,
};

enum {
	TMR_1_CMPA=0,
	TMR_1_CMPB,
	TMR_1_CMPC,
	TMR_3_CMPA,
	TMR_3_CMPB,
	TMR_3_CMPC,
	TMR_4_CMPA,
	TMR_4_CMPB,
	TMR_4_CMPC,
	TMR_5_CMPA,
	TMR_5_CMPB,
	TMR_5_CMPC,
};

//
//  error code enumerations
//
enum {
    ERR_NONE,
    ERR_BAD_PARAM,
    ERR_INVALID_MOTOR,
    ERR_DISABLED,   // motor can't move if its disabled
    ERR_IN_ENDSTOP, // motor can't move if headed towards a crash
    ERR_POSITION_UNKNOWN,   
    ERR_OUTSIDE_BOUNDARY,
	ERR_MOTORS_RUNNING,	// motor(s) are still running
};

//
//  stepControl enumerations
//
enum {
	MOTOR_DIRECTION_IN,
	MOTOR_DIRECTION_OUT,
	MOTOR_ENABLE,
	MOTOR_DISABLE,
};

//
// duration matching sort indexes
//
enum {
	ISRA=0,
	ISRB,
	ISRC,
};

//
// motor run action states - see ISR
//
enum {
	STEP_DONE = 0,		// step done must = 0
	STEP_CRUISE,
	STEP_ACCEL,
	STEP_DECEL,
	STEP_LONG,
};

//
// internal motor block structure
//
typedef struct {
	float minPosition;         	// axis minimum pos - may not be = 0
	float maxPosition;         	// axis boundary
	float curPosition;      	// last known position
	float newPosition;      	// destination position
	float speed;               	// speed in PPS
	float maxSpeed;            	// the speed limit
	float acceleration;        	// accel / decel for this motor
	bool positionKnown;       	// has motor been homed?
	volatile uint8_t mAction;  	// motor action cmd (isr state machine)
	uint16_t stepsPerMM;      	// steps / mm
	uint16_t numSteps;        	// total steps to move
	volatile uint32_t cruiseSteps;      // number of cruise speed steps to do
	uint16_t cruiseRate;       	// constant run speed (in PPS)
	volatile uint16_t rampSteps;        // number of steps in accel/decel ramp
	volatile uint16_t rampStepCount;	// keep track of steps in ramp
	volatile uint32_t cruiseInterval;   // step pulse interval at cruise speed
	volatile uint32_t cruiseReload;     // used for long period intervals
	volatile uint32_t startInterval;    // starting interval for ramp (accel seed value)
	uint32_t rampTime;        	// time of accel/decel ramp (in usec)
	uint32_t totalTime;       	// total time for this movement (in usec)

}mBlock_t;



// macro to inline this function	
#define CALC_ACCEL(P,V,S) do{ P = V * uint32_t(pgm_read_word_near(accel_lookup_table + S)); }while(0)	


// ==========================================================================
//
// jStepper class
// 
class jStepper
{
public:
	//**************************************************************
	// constructor
	//
	//
	jStepper(void);

	//**************************************************************
	// begin() imports the motor config structure and initializes all.
	// This should be done before anything else.
	//
	uint32_t begin(jsMotorConfig);

	//**************************************************************
	// setSpeed() sets the speed (movement in mm/sec) for all motors
	// given in millimeters/second.
	//
	void setSpeed(float speed0, float speed1, float speed2);

	//**************************************************************
	// getSpeed() returns the current speed 
	//
	float getSpeed(uint8_t motorNum);

	//**************************************************************
	// setMaxSpeed() sets the maximum movement in mm/sec for the motor(s)
	// given in millimeters/second.
	//
	void setMaxSpeed(float maxSpeed0, float maxSpeed1, float maxSpeed2);

	//**************************************************************
	// getMaxSpeed() returns the current max speed 
	//
	float getMaxSpeed(uint8_t motorNum);

	//**************************************************************
	// setDirection() routine to set step direction pin
	// mDir = direction, see stepControl enums
	// returns error code
	//
	void setDirection(uint8_t motorNum, uint8_t dir);

	//**************************************************************
	// getDirection() returns direction state for the given motor
	// direction - see stepControl enums
	//
	uint8_t getDirection(uint8_t motorNum);

	//**************************************************************
	// setEnabled() enables/disables the specified motor.
	// enab = true to enable
	// returns error if something wrong
	//
	void setEnabled(uint8_t motorNum, bool enab);

	//**************************************************************
	// isEnable() returns true if motor is enabled.
	//
	bool isEnabled(uint8_t motorNum);

	//**************************************************************
	// setAcceleration() sets the 'acceleration' value for the motor(s). 
	// Acceleration in this implementation creates a linear acceleration
	// profile that is expressed in steps/second^2.
	//
	void setAcceleration(float acc0, float acc1, float acc2);

	//**************************************************************
	// getAcceleration() returns current acceleration value for the
	// given motor
	//
	float getAcceleration(uint8_t motorNum);

	//**************************************************************
	// runMotors() executes the movement plan created by setPosition()
	//
	uint8_t runMotors(void);

	//**************************************************************
	// stepMotor() moves the specified motor 1 step IN or OUT depending 
	// on setDirection().
	// returns error code if step can't be performed
	//
	uint8_t stepMotor(uint8_t motorNum);

	//**************************************************************
	// isRunning() returns the run state of the given motor (true if
	// running). If motorNum == MOTOR_ALL, true is returned if ANY
	// motors are running.
	//
	bool isRunning(uint8_t motorNum);

	//**************************************************************
	// quickStop() forces an immediate halt of the given motor.
	// If motorNum == MOTOR_ALL, all motors are stopped.
	//
	void quickStop(uint8_t motorNum);

	//**************************************************************
	// setPosition() sets the absolute position for a motor in mm
	// returns error code (input neg value, out of bounds, etc.)
	//
	// This function plans motor movement, sync / async, and linear
	// acceleration / deceleration profiles.
	//
	// If mSync ==  true, all motors will synchronize to the motor
	// with the longest duration.
	//
	uint8_t setPosition(float pos0, float pos1, float pos2, bool mSync);

	//**************************************************************
	// getPosition() returns absolute position for the given motor
	//
	float getPosition(uint8_t motorNum);

	//**************************************************************
	// setPositionKnown() sets position known for the given motor
	//
	void setPositionKnown(uint8_t motorNum, bool known);

	//**************************************************************
	// isPositionKnown() gets position known for the given motor
	//
	uint8_t isPositionKnown(uint8_t motorNum);

	//**************************************************************
	// setMinPosition() sets min position for the motor group
	//
	void setMinPosition(float minPos0, float minPos1, float minPos2);  

	//**************************************************************
	// getMinPosition() returns min position for the given motor
	//
	float getMinPosition(uint8_t motorNum);     

	//**************************************************************
	// setMaxPosition() sets max position for the motor group
	//
	void setMaxPosition(float maxPos0, float maxPos1, float maxPos2);      

	//**************************************************************
	// getMaxPosition() returns max position for the given motor
	//
	float getMaxPosition(uint8_t motorNum);     

	//**************************************************************
	// timerISRn() is called by the specific timer interrupt to handle
	// step pulse generation for scheduled motors. 
	// Timer is 16 bits @ 2Mhz which means max interval count is 32768.
	//
	void timerISRA(void);
	void timerISRB(void);
	void timerISRC(void);	

	//**************************************************************
	// atMinEndStop() returns true if endstop detector triggered.
	//
	bool atMinEndStop(uint8_t motorNum);
	
	//**************************************************************
	// atMaxEndStop() returns true if endstop detector triggered.
	//
	bool atMaxEndStop(uint8_t motorNum);	

	void addTimerCallBack(uint8_t whichTimer, void *p);

	//void genLookupTable(void);    // utility routine for making new lookup tables

	void isrRedirect(uint8_t whichTimerInt);

protected:



private:
	mBlock_t mBlocks[NUM_MOTORS];
	uint16_t _sort[NUM_MOTORS];
	jsMotorConfig _mConfig;	// copy of user template

	uint16_t _TCCRA;
	uint16_t _TCCRB;
	uint16_t _TCNT;
	uint16_t _OCRA;
	uint16_t _OCRB;
	uint16_t _OCRC;
	uint16_t _TIMSK;
	uint16_t _TIFR;

};


////***************************************************************
//// used to build lookup tables
////
//void jStepper::genLookupTable(void) {
//
////
//// set up initial parameters
////
//float F = ONE_MILLION;
//float acc = 100000;
//// this is how _c0 is usually initialized
////uint32_t _c0 = round(0.676 * sqrt(2.0 / acc) * F);
//// but for this function _c0 is set to the outer bound
//uint32_t _c0 = 32768;
//uint32_t cruiseInt = 50;
//uint32_t _cn;
//float _magic = 65535;
//uint32_t i;
//
//   _cn = _c0;
//
//	for(i=1; i<=1000; i++)
//	{
//      Serial.print(uint32_t(_magic));
//		Serial.print(", ");
//		if(i % 10 == 0)
//			Serial.println("");
//
//      _cn = _cn - ((2.0 * _cn) / ((4.0 * i) + 1));
//      _cn *=  float(1.0 - (i / 82000.0));     // build S curve at the end of the ramp
//
//      _magic = float(_cn) / float(_c0);
//      _magic = uint32_t(65536.0 * _magic);
//
//      if(_cn < cruiseInt)
//         break;
//	}
//}

#endif // JSTEPPER_H
