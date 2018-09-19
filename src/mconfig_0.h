//###################################################################
//
//  jsconfig.h
// 
//  This template contains all the user configuration settings for one
//  instance of jStepper. Since ISR's are hard coded, each instance of 
//  jStepper must be derived from a separate configuration of the library. 
//  See examples.
// 

#ifndef _JSCONFIG_H
#define _JSCONFIG_H

//*******************************************************************
// TIMER SELECTION - library needs one 16 bit timer per instance
// Must be unused by any other function
//
#define USE_TIMER1
//#define USE_TIMER3
//#define USE_TIMER4
//#define USE_TIMER5


//*******************************************************************
// HARDWARE PIN ASSIGNMENTS
//
// This library assumes it is controlling a driver chip 
// like the A4888 or equivalent through the pins below.
//
// Set these values for the hardware you are supporting.
// NOTE: Not all relavent signals are dealt with by this 
// library. Example: microstepping control bits, motor current, 
// etc., will need to be set via user code.
//
// *** Arduino style pin mapping ***
//
// NOTE: comment out pins you aren't using
//
#define MOTOR_0_STEP_PIN        37
#define MOTOR_0_DIR_PIN         48
#define MOTOR_0_ENB_PIN         29

#define MOTOR_1_STEP_PIN        36
#define MOTOR_1_DIR_PIN         49
#define MOTOR_1_ENB_PIN         28 

#define MOTOR_2_STEP_PIN        35 
#define MOTOR_2_DIR_PIN         47
#define MOTOR_2_ENB_PIN         27 

#define ENDSTOP_MIN_0_PIN       12
#define ENDSTOP_MIN_1_PIN       11
#define ENDSTOP_MIN_2_PIN       10 

#define ENDSTOP_MAX_0_PIN       0
#define ENDSTOP_MAX_1_PIN       0
#define ENDSTOP_MAX_2_PIN       0

//*******************************************************************
// motor control states
//
#define STEP_PULSE_ASSERT       HIGH   // transition to this state causes step pulse
#define MOTOR_ENABLE_LEVEL      LOW    // driver enable asserted level

// direction IN means moving away from the origin (not necessarily home position)
#define MOTOR_0_DIRECTION_IN    HIGH   // each motor might define in/out differently 
#define MOTOR_1_DIRECTION_IN    LOW       
#define MOTOR_2_DIRECTION_IN    LOW     

// endstop detector active state
#define IN_ENDSTOP              HIGH 


//*******************************************************************
// Basic geometry
// motor steps per millimeter
//
#define MOTOR_0_STEPS_PER_MM	100
#define MOTOR_1_STEPS_PER_MM	100
#define MOTOR_2_STEPS_PER_MM	100


#endif // _JSCONFIG_H
