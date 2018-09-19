//###################################################################
//
//  jsconfig.h
// 
//  This template contains all the user configuration settings for one
//  instance of jStepper.
// 

#ifndef _JSCONFIG_H
#define _JSCONFIG_H

// ==========================================================================
// motor configuration structure
//
typedef struct  {
	uint8_t TIMER_SELECT;	// timer select: 1, 3, 4, 5
	uint8_t MOTOR_0_STEP_PIN;
	uint8_t MOTOR_0_DIR_PIN;
	uint8_t MOTOR_0_ENB_PIN;

	uint8_t MOTOR_1_STEP_PIN;
	uint8_t MOTOR_1_DIR_PIN;
	uint8_t MOTOR_1_ENB_PIN;

	uint8_t MOTOR_2_STEP_PIN;
	uint8_t MOTOR_2_DIR_PIN;
	uint8_t MOTOR_2_ENB_PIN;

	uint8_t  ENDSTOP_MIN_0_PIN;
	uint8_t ENDSTOP_MIN_1_PIN;
	uint8_t ENDSTOP_MIN_2_PIN;

	uint8_t ENDSTOP_MAX_0_PIN;
	uint8_t ENDSTOP_MAX_1_PIN;
	uint8_t ENDSTOP_MAX_2_PIN;

	uint8_t STEP_PULSE_ASSERT;
	uint8_t MOTOR_ENABLE_LEVEL;

	// direction IN means moving away from the origin (not necessarily home position)
	uint8_t MOTOR_0_DIRECTION_IN;   // each motor might define in/out differently
	uint8_t MOTOR_1_DIRECTION_IN;
	uint8_t MOTOR_2_DIRECTION_IN;

	// endstop detector active state
	uint8_t IN_ENDSTOP;

	// Basic geometry
	// motor steps per millimeter
	//
	uint8_t MOTOR_0_STEPS_PER_MM;
	uint8_t MOTOR_1_STEPS_PER_MM;
	uint8_t MOTOR_2_STEPS_PER_MM;
}jsMotorConfig;


#endif // _JSCONFIG_H
