/*
 *  jStepper.cpp
 */



#include "jStepper.h"


//###################################################################
// made these macros to inline code and avoid overhead
// of a function call
//
//#ifdef motor0_step_pin
#define STEP_MOTOR_0 \
		PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, _mConfig.STEP_PULSE_ASSERT); \
		DELAY_1_NOP;  \
		PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

#define STEP_MOTOR_1 \
		PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, _mConfig.STEP_PULSE_ASSERT); \
		DELAY_1_NOP; \
		PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

#define STEP_MOTOR_2 \
		PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, _mConfig.STEP_PULSE_ASSERT); \
		DELAY_1_NOP; \
		PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

#define STEP_MOTOR(MOTNUM) do{ \
	switch (MOTNUM) { \
	case MOTOR_0: STEP_MOTOR_0 \
		break; \
	case MOTOR_1: STEP_MOTOR_1 \
		break; \
	case MOTOR_2: STEP_MOTOR_2 \
		break; \
	} }while(0)

	// *** Timer ISR dynamic allocation ***
	// Each instance of the library will claim one timer and install it's instance in
	// one of these object pointers. This is how the ISR redirects to the correct instance.
	// If the pointer is NULL and an interrupt occurs the isrRedirectArray is checked
	// for a user installed external handler. If no pointer, the interrupt simply returns.
	//
    jStepper *psPtr_1 = NULL;
    jStepper *psPtr_3 = NULL;
    jStepper *psPtr_4 = NULL;
    jStepper *psPtr_5 = NULL;

    typedef void(*functPtr)(void);			// typedef a void func pointer
    jStepper *isrRedirectObj = NULL;		// owner will be the first lib instance

    // global array used to store user supplied external interrupt handlers
    static functPtr isrRedirectArray[17] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

//###################################################################
//
// jStepper constructor
jStepper::jStepper(void)
{
	//_extCallBack = extCallBack;
}


//###################################################################
//
uint8_t jStepper::begin(jsMotorConfig mC)
{
	uint8_t i;

	if(sizeof(mC) == 0)
		return ERR_NULL_PTR;

	_mConfig = mC;		// keep a local copy of the user struct

	if(isrRedirectObj == NULL)
		isrRedirectObj = this;	// only init one time

	mBlocks[0].stepsPerUnit = _mConfig.MOTOR_0_STEPS_PER_MM;
	mBlocks[1].stepsPerUnit = _mConfig.MOTOR_1_STEPS_PER_MM;
	mBlocks[2].stepsPerUnit = _mConfig.MOTOR_2_STEPS_PER_MM;
			Serial.print("steps/mm=");
			Serial.println(mBlocks[0].stepsPerUnit);

	//***************************************************************
	// initialize I/O pins
	//
	// output pins ---
	//
	PIN_OUTPUT(_mConfig.MOTOR_0_ENB_PIN);
	PIN_WRITE(_mConfig.MOTOR_0_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);

	PIN_OUTPUT(_mConfig.MOTOR_1_ENB_PIN);
	PIN_WRITE(_mConfig.MOTOR_1_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);

	PIN_OUTPUT(_mConfig.MOTOR_2_ENB_PIN);
	PIN_WRITE(_mConfig.MOTOR_2_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);

	PIN_OUTPUT(_mConfig.MOTOR_0_STEP_PIN);
	PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

	PIN_OUTPUT(_mConfig.MOTOR_1_STEP_PIN);
	PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

	PIN_OUTPUT(_mConfig.MOTOR_2_STEP_PIN);
	PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);

	PIN_OUTPUT(_mConfig.MOTOR_0_DIR_PIN);
	PIN_OUTPUT(_mConfig.MOTOR_1_DIR_PIN);
	PIN_OUTPUT(_mConfig.MOTOR_2_DIR_PIN);

	//
	// input pins
	//
	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MIN_0_PIN);
	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MIN_1_PIN);
	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MIN_2_PIN);

	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MAX_0_PIN);	// set up endstop input w/pullup
	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MAX_1_PIN);
	PIN_INPUT_PULLUP(_mConfig.ENDSTOP_MAX_2_PIN);

	//
	// this is where the timer regs are dynamically allocated
	// based on user selected timer: 1, 3, 4, or 5
	//
	switch(_mConfig.TIMER_SELECT)
	{
		case TIMER_SEL_1:
			_TCCRA = _TCCR1A;
			_TCCRB = _TCCR1B;
			_TCNT = _TCNT1;
			_OCRA = _OCR1A;
			_OCRB = _OCR1B;
			_OCRC = _OCR1C;
			_TIMSK = _TIMSK1;
			_TIFR = _TIFR1;

			psPtr_1 = this;			// set ISR object ptr to this instance
			break;

		case TIMER_SEL_3:
			_TCCRA = _TCCR3A;
			_TCCRB = _TCCR3B;
			_TCNT = _TCNT3;
			_OCRA = _OCR3A;
			_OCRB = _OCR3B;
			_OCRC = _OCR3C;
			_TIMSK = _TIMSK3;
			_TIFR = _TIFR3;

			psPtr_3 = this;
			break;

		case TIMER_SEL_4:
			_TCCRA = _TCCR4A;
			_TCCRB = _TCCR4B;
			_TCNT = _TCNT4;
			_OCRA = _OCR4A;
			_OCRB = _OCR4B;
			_OCRC = _OCR4C;
			_TIMSK = _TIMSK4;
			_TIFR = _TIFR4;

			psPtr_4 = this;
			break;

		case TIMER_SEL_5:
			_TCCRA = _TCCR5A;
			_TCCRB = _TCCR5B;
			_TCNT = _TCNT5;
			_OCRA = _OCR5A;
			_OCRB = _OCR5B;
			_OCRC = _OCR5C;
			_TIMSK = _TIMSK5;
			_TIFR = _TIFR5;

			psPtr_5 = this;
			break;

		default:
			return ERR_BAD_PARAM;
			break;
	}

	//
	// set default parameters for all motors
	//
	for (i = MOTOR_0; i < NUM_MOTORS; i++)
	{
		mBlocks[i].curPosition = 0.0;
		mBlocks[i].newPosition = 0.0;
		mBlocks[i].maxSpeed = MAX_SPEED;
		mBlocks[i].cruiseSteps = 0;
		mBlocks[i].numSteps = 0;
		mBlocks[i].positionKnown = false;
		mBlocks[i].mAction = STEP_DONE;
	}

	mBlocks[MOTOR_0].minPosition = _mConfig.MOTOR_0_MINPOS;
	mBlocks[MOTOR_0].maxPosition = _mConfig.MOTOR_0_MAXPOS;
	mBlocks[MOTOR_0].speed = _mConfig.MOTOR_0_SPEED;
	mBlocks[MOTOR_0].acceleration = _mConfig.MOTOR_0_ACCEL;
	mBlocks[MOTOR_0].homeInvert = _mConfig.MOTOR_0_HOMING_INVERT;
	mBlocks[MOTOR_0].positionMode = MODE_ABSOLUTE;

	mBlocks[MOTOR_1].minPosition = _mConfig.MOTOR_1_MINPOS;
	mBlocks[MOTOR_1].maxPosition = _mConfig.MOTOR_1_MAXPOS;
	mBlocks[MOTOR_1].speed = _mConfig.MOTOR_1_SPEED;
	mBlocks[MOTOR_1].acceleration = _mConfig.MOTOR_1_ACCEL;
	mBlocks[MOTOR_1].homeInvert = _mConfig.MOTOR_1_HOMING_INVERT;
	mBlocks[MOTOR_1].positionMode = MODE_ABSOLUTE;

	mBlocks[MOTOR_2].minPosition = _mConfig.MOTOR_2_MINPOS;
	mBlocks[MOTOR_2].maxPosition = _mConfig.MOTOR_2_MAXPOS;
	mBlocks[MOTOR_2].speed = _mConfig.MOTOR_2_SPEED;
	mBlocks[MOTOR_2].acceleration = _mConfig.MOTOR_2_ACCEL;
	mBlocks[MOTOR_2].homeInvert = _mConfig.MOTOR_2_HOMING_INVERT;
	mBlocks[MOTOR_2].positionMode = MODE_ABSOLUTE;

	_planValid = false;				// no movement plans yet

	return ERR_NONE;
}


//###################################################################
// setAcceleration()
//
void jStepper::setAcceleration(float acc0, float acc1, float acc2)
{ 
	if(acc0 >= 0.0)	// no change if neg
		mBlocks[MOTOR_0].acceleration = (acc0 > MAX_ACCELERATION) ? MAX_ACCELERATION : acc0;
   
	if(acc1 >= 0.0)
		mBlocks[MOTOR_1].acceleration = (acc1 > MAX_ACCELERATION) ? MAX_ACCELERATION : acc1;
   
	if(acc2 >= 0.0)	
		mBlocks[MOTOR_2].acceleration = (acc2 > MAX_ACCELERATION) ? MAX_ACCELERATION : acc2;

}


//###################################################################
// getAcceleration()
//
float jStepper::getAcceleration(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].acceleration;
	else
		return -1;
}

//###################################################################
//
// set motor speed in mm / min.
//
void jStepper::setSpeed(float speed0, float speed1, float speed2)   // speed in mm/sec
{		
	if(speed0 > 0.0)	// no change if <= 0.0
	{
		mBlocks[MOTOR_0].speed = (speed0 > mBlocks[MOTOR_0].maxSpeed) ? mBlocks[MOTOR_0].maxSpeed : speed0;
	}

	if(speed1 > 0.0)		
	{
		mBlocks[MOTOR_1].speed = (speed1 > mBlocks[MOTOR_1].maxSpeed) ? mBlocks[MOTOR_1].maxSpeed : speed1;
	}
	
	if(speed2 > 0.0)		
	{
		mBlocks[MOTOR_2].speed = (speed2 > mBlocks[MOTOR_2].maxSpeed) ? mBlocks[MOTOR_2].maxSpeed : speed2;
	}
}


//###################################################################
//
float jStepper::getSpeed(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].speed;
	else
		return -1;
}


//###################################################################
//
// set motor speed in mm / sec.
void jStepper::setMaxSpeed(float maxSpeed0, float maxSpeed1, float maxSpeed2)   // speed in mm/sec
{
      mBlocks[MOTOR_0].maxSpeed = abs(maxSpeed0);
      mBlocks[MOTOR_1].maxSpeed = abs(maxSpeed1);
      mBlocks[MOTOR_2].maxSpeed = abs(maxSpeed2);
}


//###################################################################
//
float jStepper::getMaxSpeed(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].maxSpeed;
	else
		return -1;
}


//###################################################################
//
// returns true if motors are running
//
bool jStepper::isRunning(uint8_t motorNum) 
{
   uint8_t b = 0;
   
   if(motorNum == 0 || motorNum == MOTOR_ALL)
      b += mBlocks[MOTOR_0].mAction;      // step done if == 0
      
   if(motorNum == 1 || motorNum == MOTOR_ALL)
      b += mBlocks[MOTOR_1].mAction;
            
   if(motorNum == 2 || motorNum == MOTOR_ALL)
      b += mBlocks[MOTOR_2].mAction;
   
   return (b > 0);
}



//###################################################################
// Called from isr to report step completion and to call external
// callback function if no motors are running.
// Interrupts are enabled to prevent blocking other timer interrupts.
//
void jStepper::stepComplete(uint8_t motorNum)
{
	// void func pointer for external callbacks
	mBlocks[motorNum].mAction = STEP_DONE;
	if(!isRunning(MOTOR_ALL) && isrRedirectArray[STEP_COMPLETE_CALLBACK] != NULL)
	{
		//sei();		// prevent user function from blocking other interrupts
		(*(isrRedirectArray[STEP_COMPLETE_CALLBACK]))();
	}
}


//###################################################################
//
void jStepper::quickStop(uint8_t motorNum) 
{
	if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
	{
		setEnabled(MOTOR_0, false);
		mBlocks[MOTOR_0].mAction = STEP_DONE;
	}
	if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
	{
		setEnabled(MOTOR_1, false);
	    mBlocks[MOTOR_1].mAction = STEP_DONE;
	}
	if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
	{
		setEnabled(MOTOR_2, false);
	    mBlocks[MOTOR_2].mAction = STEP_DONE;
	}
}


//###################################################################
//
// setPosition() sets the given motors absolute position regardless
// of it's actual location.
//
uint8_t jStepper::setPosition(uint8_t motorNum, float newPos)
{
	if(motorNum < NUM_MOTORS)
	{
		mBlocks[motorNum].curPosition = newPos;
		return ERR_NONE;
	}
	else
		return ERR_INVALID_MOTOR;
}


//###################################################################
//
float jStepper::getPosition(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].curPosition;
	else
		return -1;
}

//###################################################################
// get actual position during motor movement
float jStepper::getPositionActual(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
	{
		if (!isRunning(motorNum))
		{
			return mBlocks[motorNum].curPosition;
		}
		else
		{
			switch (mBlocks[MOTOR_0].mAction)  // check profile state
			{
				case STEP_CRUISE:		// constant speed
				case STEP_LONG:

					if (!getDirection(motorNum))
						return mBlocks[motorNum].curPosition - (float) mBlocks[motorNum].rampStepCount / mBlocksNext[motorNum].stepsPerUnit - (float) mBlocks[motorNum].cruiseSteps / mBlocksNext[motorNum].stepsPerUnit;
					else
						return mBlocks[motorNum].curPosition - (-1 * (float) mBlocks[motorNum].rampStepCount / mBlocksNext[motorNum].stepsPerUnit - (float) mBlocks[motorNum].cruiseSteps / mBlocksNext[motorNum].stepsPerUnit);
				
				case STEP_ACCEL:		// accelerating
					if (!getDirection(motorNum))
						return mBlocks[motorNum].curPosition - (float) mBlocks[motorNum].cruiseSteps / mBlocksNext[motorNum].stepsPerUnit - (float) mBlocks[motorNum].rampSteps / mBlocksNext[motorNum].stepsPerUnit - (float) (mBlocks[motorNum].rampSteps - mBlocks[motorNum].rampStepCount) / mBlocksNext[motorNum].stepsPerUnit;
					else
						return mBlocks[motorNum].curPosition - (-1 * (float) mBlocks[motorNum].cruiseSteps / mBlocksNext[motorNum].stepsPerUnit - (float) mBlocks[motorNum].rampSteps / mBlocksNext[motorNum].stepsPerUnit - (float) (mBlocks[motorNum].rampSteps - mBlocks[motorNum].rampStepCount) / mBlocksNext[motorNum].stepsPerUnit);
				
				case STEP_DECEL:		// decelerating
					if (!getDirection(motorNum))
						return mBlocks[motorNum].curPosition - (float) mBlocks[motorNum].rampStepCount / (float) mBlocksNext[motorNum].stepsPerUnit;
					else
						return mBlocks[motorNum].curPosition - (-1 * (float) mBlocks[motorNum].rampStepCount / (float) mBlocksNext[motorNum].stepsPerUnit);

				default:
					return 0;
			}
		}
	}
	else
	{
		return -1;
	}
}

//###################################################################
// setPositionMode() sets absolute or relative positioning
// Absolute (default) - all moves are relative to the origin.
// Relative - all moves are relative to the last position.
// If motorNum == MOTOR_ALL, all motors are set to the same
// positioning mode.
//
void jStepper::setPositionMode(uint8_t motorNum, uint8_t positionMode)
{
	if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
	{
		mBlocks[MOTOR_0].positionMode = positionMode;
	}
	if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
	{
		mBlocks[MOTOR_1].positionMode = positionMode;
	}
	if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
	{
		mBlocks[MOTOR_2].positionMode = positionMode;
	}
}


//###################################################################
// getPositionMode() gets positioning mode for the requested motor.
//
uint8_t jStepper::getPositionMode(uint8_t motorNum)
{
	return mBlocks[motorNum].positionMode;
}


//###################################################################
// setMinPosition() sets min position for the given motor
//
void jStepper::setMinPosition(float minPos0, float minPos1, float minPos2) 
{
    mBlocks[MOTOR_0].minPosition = minPos0;
    mBlocks[MOTOR_1].minPosition = minPos1;
    mBlocks[MOTOR_2].minPosition = minPos2;    
}
  

//###################################################################
// returns -1 if error
float jStepper::getMinPosition(uint8_t motorNum)
{
	if(motorNum >= NUM_MOTORS)
		return -1;
	else
		return mBlocks[motorNum].minPosition;
}


//###################################################################
// setMaxPosition() sets max position for the given motor
//
void jStepper::setMaxPosition(float maxPos0, float maxPos1, float maxPos2) 
{
    mBlocks[MOTOR_0].maxPosition = maxPos0;
    mBlocks[MOTOR_1].maxPosition = maxPos1;    
    mBlocks[MOTOR_2].maxPosition = maxPos2;   
}


//###################################################################
// returns -1 if error
//
float jStepper::getMaxPosition(uint8_t motorNum)
{
	if(motorNum >= NUM_MOTORS)
		return -1;
	else
		return mBlocks[motorNum].maxPosition;
}


//###################################################################
//
void jStepper::setDirection(uint8_t motorNum, uint8_t dir)
{
	switch(motorNum)
	{
		case MOTOR_0:
			if (dir == MOTOR_DIRECTION_IN)
				PIN_WRITE(_mConfig.MOTOR_0_DIR_PIN, _mConfig.MOTOR_0_DIRECTION_IN);
			else
				PIN_WRITE(_mConfig.MOTOR_0_DIR_PIN, !_mConfig.MOTOR_0_DIRECTION_IN);
			break;
		
		case MOTOR_1:
			if (dir == MOTOR_DIRECTION_IN)
				PIN_WRITE(_mConfig.MOTOR_1_DIR_PIN, _mConfig.MOTOR_1_DIRECTION_IN);
			else
				PIN_WRITE(_mConfig.MOTOR_1_DIR_PIN, !_mConfig.MOTOR_1_DIRECTION_IN);
			break;
			
		case MOTOR_2:
			if (dir == MOTOR_DIRECTION_IN)
				PIN_WRITE(_mConfig.MOTOR_2_DIR_PIN, _mConfig.MOTOR_2_DIRECTION_IN);
			else
				PIN_WRITE(_mConfig.MOTOR_2_DIR_PIN, !_mConfig.MOTOR_2_DIRECTION_IN);
			break;			
	}
}


//###################################################################
//
uint8_t jStepper::getDirection(uint8_t motorNum)
{
	switch (motorNum) 
    {
	    case MOTOR_0:
            if(PIN_READ(_mConfig.MOTOR_0_DIR_PIN) == _mConfig.MOTOR_0_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
            break;

	    case MOTOR_1:
            if(PIN_READ(_mConfig.MOTOR_1_DIR_PIN) == _mConfig.MOTOR_1_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
		    break;

	    case MOTOR_2:
            if(PIN_READ(_mConfig.MOTOR_2_DIR_PIN) == _mConfig.MOTOR_2_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
		    break;
	}
	return 0;
}


//###################################################################
//
void jStepper::setEnabled(uint8_t motorNum, bool enab)
{
	if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
	{
		if (enab)
			PIN_WRITE(_mConfig.MOTOR_0_ENB_PIN, _mConfig.MOTOR_ENABLE_LEVEL);
		else
			PIN_WRITE(_mConfig.MOTOR_0_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);
	}

	if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
	{
		if (enab)
			PIN_WRITE(_mConfig.MOTOR_1_ENB_PIN, _mConfig.MOTOR_ENABLE_LEVEL);
		else
			PIN_WRITE(_mConfig.MOTOR_1_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);
	}

	if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
	{
		if (enab)
			PIN_WRITE(_mConfig.MOTOR_2_ENB_PIN, _mConfig.MOTOR_ENABLE_LEVEL);
		else
			PIN_WRITE(_mConfig.MOTOR_2_ENB_PIN, !_mConfig.MOTOR_ENABLE_LEVEL);
	}
}


//###################################################################
//
bool jStepper::isEnabled(uint8_t motorNum)
{
    switch(motorNum)
    {
        case MOTOR_0:
            return (PIN_READ(_mConfig.MOTOR_0_ENB_PIN) == _mConfig.MOTOR_ENABLE_LEVEL);
            break;

        case MOTOR_1:
           	return (PIN_READ(_mConfig.MOTOR_1_ENB_PIN) == _mConfig.MOTOR_ENABLE_LEVEL);
            break;

        case MOTOR_2:
           	return (PIN_READ(_mConfig.MOTOR_2_ENB_PIN) == _mConfig.MOTOR_ENABLE_LEVEL);
            break;

        default:
        	return false;
        	break;
    }
}


//###################################################################
// move motor one step towards the last setDirection() call
//
uint8_t jStepper::stepMotor(uint8_t motorNum)
{
    if(!isEnabled(motorNum))    // motor enabled?
        return ERR_DISABLED;

    if(motorNum >= NUM_MOTORS)
        return ERR_INVALID_MOTOR;   // invalid motor number?

    STEP_MOTOR(motorNum);
    return ERR_NONE;
}


//###################################################################
// returns true if endstop detected
//
bool jStepper::atMinEndStop(uint8_t motorNum) {
	switch (motorNum) 
    {
	    case MOTOR_0:
	    	return (PIN_READ(_mConfig.ENDSTOP_MIN_0_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    case MOTOR_1:
	    	return (PIN_READ(_mConfig.ENDSTOP_MIN_1_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    case MOTOR_2:
	    	return (PIN_READ(_mConfig.ENDSTOP_MIN_2_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    default:
	    	return false;
	    	break;
	}
}


//###################################################################
// returns true if endstop detected
//
bool jStepper::atMaxEndStop(uint8_t motorNum) {
	switch (motorNum) 
    {
	    case MOTOR_0:
    		return (PIN_READ(_mConfig.ENDSTOP_MAX_0_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    case MOTOR_1:
	    	return (PIN_READ(_mConfig.ENDSTOP_MAX_1_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    case MOTOR_2:
	    	return (PIN_READ(_mConfig.ENDSTOP_MAX_2_PIN) == _mConfig.IN_ENDSTOP);
		    break;

	    default:
	    	return false;
	    	break;
	}
}


//###################################################################
//
void jStepper::setPositionKnown(uint8_t motorNum, bool known)
{
    if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
    	mBlocks[MOTOR_0].positionKnown = known;

    if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
    	mBlocks[MOTOR_1].positionKnown = known;

    if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
    	mBlocks[MOTOR_2].positionKnown = known;
}


//###################################################################
//
uint8_t jStepper::isPositionKnown(uint8_t motorNum)
{
    if(motorNum < NUM_MOTORS)
    	return mBlocks[motorNum].positionKnown;
    else
        return false;
}


//###################################################################
//
void jStepper::timerISRA(void)
{
	union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
	
	switch (mBlocks[MOTOR_0].mAction)     // check profile state
	{
    	case STEP_CRUISE:			// constant speed
    		PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);		// step pulse assert
    		OCRA += uint16_t(mBlocks[MOTOR_0].cruiseInterval << 1);
			if (--mBlocks[MOTOR_0].cruiseSteps == 0)
			{
			   if(mBlocks[MOTOR_0].rampSteps)
				   mBlocks[MOTOR_0].mAction = STEP_DECEL;
			   else
			   {
				   CLRb(TIMSK, OCIEA);  // all done
				   stepComplete(MOTOR_0);
				   //mBlocks[MOTOR_0].mAction = STEP_DONE;
			   }
			}

			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);	// step pulse deassert
			break;

		case STEP_ACCEL:		// accelerating
			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_0].startInterval, ++mBlocks[MOTOR_0].rampStepCount);
			OCRA += pun.val16[1] << 1;
			if (mBlocks[MOTOR_0].rampStepCount >= mBlocks[MOTOR_0].rampSteps)
			   mBlocks[MOTOR_0].mAction = (mBlocks[MOTOR_0].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_0].startInterval, mBlocks[MOTOR_0].rampStepCount--);
			OCRA += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_0].rampStepCount)
			{
			   CLRb(TIMSK, OCIEA);  // last phase, all done!
			   stepComplete(MOTOR_0);
			   //mBlocks[MOTOR_0].mAction = STEP_DONE;
			}
			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;
			
      case STEP_LONG:			// long interval
         if(mBlocks[MOTOR_0].cruiseInterval < 32768)
         {
        	PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
            mBlocks[MOTOR_0].cruiseInterval = mBlocks[MOTOR_0].cruiseReload;
            frac = mBlocks[MOTOR_0].cruiseInterval & 0x7FFF;
            NOTLESS(frac, 16);	// move fwd enough so we don't overrun timer
            OCRA = OCRA + (frac << 1);  // move fractional part of interval ahead
			if (--mBlocks[MOTOR_0].cruiseSteps == 0)
			{
               CLRb(TIMSK, OCIEA);  // disable int's
               stepComplete(MOTOR_0);
  		       //mBlocks[MOTOR_0].mAction = STEP_DONE;
            }
            else
               SETb(TIFR, OCFA);    	// clear any pending interrupts
			PIN_WRITE(_mConfig.MOTOR_0_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
         }
         else
            mBlocks[MOTOR_0].cruiseInterval -= 32768;    // interval minus whole number

         break;			
	}   // end switch()
}


//###################################################################
//
void jStepper::timerISRB(void)
{
	union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
		
   switch (mBlocks[MOTOR_1].mAction)     // check profile state
   {
      case STEP_CRUISE:			// constant speed
    	  PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
    	  OCRB += (mBlocks[MOTOR_1].cruiseInterval << 1);
    	  if (--mBlocks[MOTOR_1].cruiseSteps == 0)
    	  {
    		  if(mBlocks[MOTOR_1].rampSteps)
    			  mBlocks[MOTOR_1].mAction = STEP_DECEL;
    		  else
    		  {
    			  CLRb(TIMSK, OCIEB);  // all done
    			  stepComplete(MOTOR_1);
    			  //mBlocks[MOTOR_1].mAction = STEP_DONE;
    		  }
    	  }	
    	  PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
    	  break;

		case STEP_ACCEL:		// accelerating
			PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_1].startInterval, ++mBlocks[MOTOR_1].rampStepCount);
			OCRB += pun.val16[1] << 1;
			if (mBlocks[MOTOR_1].rampStepCount >= mBlocks[MOTOR_1].rampSteps)
			   mBlocks[MOTOR_1].mAction = (mBlocks[MOTOR_1].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_1].startInterval, mBlocks[MOTOR_1].rampStepCount--);
			OCRB += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_1].rampStepCount)
			{
			   CLRb(TIMSK, OCIEB);  // last phase, all done!
			   stepComplete(MOTOR_1);
			   //mBlocks[MOTOR_1].mAction = STEP_DONE;
			}
			PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;
			
      case STEP_LONG:			// long interval
         if(mBlocks[MOTOR_1].cruiseInterval < 32768)
         {
        	 PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
        	 mBlocks[MOTOR_1].cruiseInterval = mBlocks[MOTOR_1].cruiseReload;
        	 frac = mBlocks[MOTOR_1].cruiseInterval & 0x7FFF;
        	 NOTLESS(frac, 16);
        	 OCRB = OCRB + (frac << 1);  
			 	 if (--mBlocks[MOTOR_1].cruiseSteps == 0)
			 	 {
			 		 //mBlocks[MOTOR_1].mAction = STEP_DONE;
			 		 CLRb(TIMSK, OCIEB);  // disable int's
			 		stepComplete(MOTOR_1);
			 	 }
			 	 else
			 		 SETb(TIFR, OCFB);    	// clear pending interrupts
			 	PIN_WRITE(_mConfig.MOTOR_1_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
         }
         else
            mBlocks[MOTOR_1].cruiseInterval -= 32768;    // interval - whole number
         break;				
	}   // end switch()
}


//###################################################################
//
void jStepper::timerISRC(void)
{
	union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
		
   switch (mBlocks[MOTOR_2].mAction)     // check profile state
   {
      case STEP_CRUISE:			// constant speed
    	  PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
    	  OCRC += (mBlocks[MOTOR_2].cruiseInterval << 1);
    	  if (--mBlocks[MOTOR_2].cruiseSteps == 0)
    	  {
    		  if(mBlocks[MOTOR_2].rampSteps)
    			  mBlocks[MOTOR_2].mAction = STEP_DECEL;
    		  else
    		  {
    			  CLRb(TIMSK, OCIEC);  // all done
    			  stepComplete(MOTOR_2);
     		      //mBlocks[MOTOR_2].mAction = STEP_DONE;
    		  }
    	  }	
    	  PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
    	  break;

		case STEP_ACCEL:		// accelerating
			PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);;
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_2].startInterval, ++mBlocks[MOTOR_2].rampStepCount);
			OCRC += pun.val16[1] << 1;
			if (mBlocks[MOTOR_2].rampStepCount >= mBlocks[MOTOR_2].rampSteps)
			   mBlocks[MOTOR_2].mAction = (mBlocks[MOTOR_2].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_2].startInterval, mBlocks[MOTOR_2].rampStepCount--);
			OCRC += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_2].rampStepCount)
			{
			   CLRb(TIMSK, OCIEC);  // last phase, all done!
			   stepComplete(MOTOR_2);
			   //mBlocks[MOTOR_2].mAction = STEP_DONE;
			}
			PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			break;
			
		case STEP_LONG:			// long interval
			if(mBlocks[MOTOR_2].cruiseInterval < 32768)
			{
				PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, _mConfig.STEP_PULSE_ASSERT);
				mBlocks[MOTOR_2].cruiseInterval = mBlocks[MOTOR_2].cruiseReload;
				frac = mBlocks[MOTOR_2].cruiseInterval & 0x7FFF;
				NOTLESS(frac, 16);
				OCRC = OCRC + (frac << 1);  // move fractional part of interval ahead
				if (--mBlocks[MOTOR_2].cruiseSteps == 0)
				{
					//mBlocks[MOTOR_2].mAction = STEP_DONE;
					CLRb(TIMSK, OCIEC);  // disable int's
					stepComplete(MOTOR_2);
				}
				else
					SETb(TIFR, OCFC);    	// clear pending interrupts
				PIN_WRITE(_mConfig.MOTOR_2_STEP_PIN, !_mConfig.STEP_PULSE_ASSERT);
			}
			else
				mBlocks[MOTOR_2].cruiseInterval -= 32768;    // interval - whole number
			break;		
	}   // end switch()
}


//###################################################################
//
uint8_t jStepper::homeMotor(uint8_t motorNum, uint16_t hSpeed)	// speed is in mm/sec
{
	if(isRunning(MOTOR_ALL))
		return ERR_MOTORS_RUNNING;		// busy running something!

	mBlocks[motorNum].mAction = HOMING_PHASE1;
	mBlocks[motorNum].lastResult = ERR_NONE;
	_homingSpeed = 1000000 / (hSpeed * mBlocks[motorNum].stepsPerUnit);
	_homingSteps = 0;
	_homingMotor = motorNum;			// remember the motor to run
	_maxHomingSteps = mBlocks[_homingMotor].maxPosition * mBlocks[_homingMotor].stepsPerUnit; 	// max steps to try

	// fire off timer
	//TCNT = 0x00;
	TIMSK = 0x0;
	TCCRA = 0x0;			// normal mode, 2mhz clk
	TCCRB = 0x02;
	TCNT = 65535 - (hSpeed * 2);	// starting speed in usec
	SETb(TIMSK, TOIE);		// enable timer overflow interrupt
	SETb(TIFR, TOV);   		// clear any pending interrupts

	return ERR_NONE;
}
 

//###################################################################
//
void jStepper::homeISR(void)
{
	// uint32_t i = 0;

	switch(mBlocks[_homingMotor].mAction)
	{
		case HOMING_PHASE1:
			// head toward endstop at nominal speed
			if(mBlocks[_homingMotor].homeInvert)
				setDirection(_homingMotor, MOTOR_DIRECTION_IN);
			else
				setDirection(_homingMotor, MOTOR_DIRECTION_OUT);

			if(!atMinEndStop(_homingMotor))		// endstop detected yet?
			{
				mBlocks[_homingMotor].lastResult = stepMotor(_homingMotor);	// do one step

				if(_homingSteps++ >= _maxHomingSteps)		// too many steps?
					mBlocks[_homingMotor].lastResult = ERR_ENDSTOP_NOT_FOUND;

				if(mBlocks[_homingMotor].lastResult != ERR_NONE)
				{
					TIMSK = 0x0;	// kill all timer interrupts
					stepComplete(_homingMotor);
				}
				TCNT = 65535 - (_homingSpeed * 2);		// reset timer for next cycle
			}
			else
			{
				mBlocks[_homingMotor].mAction = HOMING_PHASE2;
				_homingSteps = 0;
				TCNT = 65535 - (_homingSpeed * 8);		// 1/4 speed for next phase
			}
			break;

		case HOMING_PHASE2:
			// back up a few clicks outside of endstop
			if(mBlocks[_homingMotor].homeInvert)
				setDirection(_homingMotor, MOTOR_DIRECTION_OUT);
			else
				setDirection(_homingMotor, MOTOR_DIRECTION_IN);

			if(_homingSteps++ < (mBlocks[_homingMotor].stepsPerUnit * 2))	// move out 2 units (mm)
			{
				mBlocks[_homingMotor].lastResult = stepMotor(_homingMotor);

				if(mBlocks[_homingMotor].lastResult != ERR_NONE)
				{
					TIMSK = 0x0;	// kill interrupts
					stepComplete(_homingMotor);
					break;
				}
				TCNT = 65535 - (_homingSpeed * 8);	// 1/4 speed
			}
			else
			{
				mBlocks[_homingMotor].mAction = HOMING_PHASE3;
				_homingSteps = 0;
				TCNT = 65535 - (_homingSpeed * 16);	// 1/8 speed
			}
			break;

		case HOMING_PHASE3:
			if(mBlocks[_homingMotor].homeInvert)
				setDirection(_homingMotor, MOTOR_DIRECTION_IN);
			else
				setDirection(_homingMotor, MOTOR_DIRECTION_OUT);

			if(_homingSteps++ > (mBlocks[_homingMotor].stepsPerUnit * 4))	// step in max of 4 units (mm)
				mBlocks[_homingMotor].lastResult = ERR_ENDSTOP_NOT_FOUND;

			if(!atMinEndStop(_homingMotor) && mBlocks[_homingMotor].lastResult == ERR_NONE)		// endstop detected yet?
			{
				mBlocks[_homingMotor].lastResult = stepMotor(_homingMotor);	// do one step
				TCNT = 65535 - (_homingSpeed * 16);		// reset timer for next cycle
			}
			else
			{
				TIMSK = 0x0;	// kill interrupts
				setPositionKnown(_homingMotor, true);

				if(!mBlocks[_homingMotor].homeInvert)
					setPosition(_homingMotor, getMinPosition(_homingMotor));
				else
					setPosition(_homingMotor, getMaxPosition(_homingMotor));

				stepComplete(_homingMotor);			// end of movement, call user callback if set
			}
			break;
		}
}


//###################################################################
//
uint8_t jStepper::planMoves(float pos0, float pos1, float pos2, bool mSync)
{
	bool swap;
	uint32_t i;
	uint32_t startT;
	uint32_t endT;
	uint32_t rampSteps;
	uint32_t sTot = 0;
	uint32_t tTot;
	bool nothingToDo = true;
	float fr, P0, P1, P2;
	union {
	   uint16_t val16[2];
	   uint32_t val32;
	}pun;

	memcpy(&mBlocksNext, &mBlocks, sizeof(mBlocks));	// use temporary structure to do planning
	_planResult = ERR_NONE;

	//
	// absolute or relative positioning?
	//
	P0 = (mBlocksNext[MOTOR_0].positionMode == MODE_ABSOLUTE) ? pos0 : pos0 + mBlocksNext[MOTOR_0].curPosition;
	P1 = (mBlocksNext[MOTOR_1].positionMode == MODE_ABSOLUTE) ? pos1 : pos1 + mBlocksNext[MOTOR_1].curPosition;
	P2 = (mBlocksNext[MOTOR_2].positionMode == MODE_ABSOLUTE) ? pos2 : pos2 + mBlocksNext[MOTOR_2].curPosition;

	//
	// do a sanity check on new position values
	//
    if(P0 < mBlocksNext[MOTOR_0].minPosition || P0 > mBlocksNext[MOTOR_0].maxPosition)
    	_planResult = ERR_OUTSIDE_BOUNDARY;
    else
    	mBlocksNext[MOTOR_0].newPosition = P0;

    if(P1 < mBlocksNext[MOTOR_1].minPosition || P1 > mBlocksNext[MOTOR_1].maxPosition)
    	_planResult = ERR_OUTSIDE_BOUNDARY;
    else
    	mBlocksNext[MOTOR_1].newPosition = P1;

    if(P2 < mBlocksNext[MOTOR_2].minPosition || P2 > mBlocksNext[MOTOR_2].maxPosition)
    	_planResult = ERR_OUTSIDE_BOUNDARY;
    else
    	mBlocksNext[MOTOR_2].newPosition = P2;

    if(_planResult != ERR_NONE)
    	return _planResult;

    //
	// iterate through all motors
    //
	for (i = 0; i < NUM_MOTORS; i++)
	{
		//
		// calc number of steps & direction of movement
		//
		fr = mBlocksNext[i].newPosition - mBlocksNext[i].curPosition;
        fr = fabs(fr);								// flip to positive num
		mBlocksNext[i].numSteps = round(fr * mBlocksNext[i].stepsPerUnit);
		mBlocksNext[i].cruiseSteps = mBlocksNext[i].numSteps; 	// initialize cruise steps
		mBlocksNext[i].mAction = STEP_DONE;						// clear action

		if (mBlocksNext[i].numSteps > 0)
		{
			nothingToDo = false;					// found something todo

			//
			// calc feedrate and duration
			//
			fr = roundf(mBlocksNext[i].speed * mBlocksNext[i].stepsPerUnit);   // calc cruise rate in PPS
			mBlocksNext[i].cruiseRate = (fr > MAX_CRUISE_RATE) ? MAX_CRUISE_RATE : fr;
			mBlocksNext[i].cruiseInterval = (1.0 / mBlocksNext[i].cruiseRate) * ONE_MILLION;
			mBlocksNext[i].cruiseReload = mBlocksNext[i].cruiseInterval;
			mBlocksNext[i].startInterval = mBlocksNext[i].cruiseInterval;

			//
			// calculate duration of movement
			//
			mBlocksNext[i].totalTime = uint32_t(mBlocksNext[i].cruiseInterval) * uint32_t(mBlocksNext[i].numSteps);
		}
		else	// no movement on this motor
		{
			mBlocksNext[i].totalTime = 0;
			mBlocksNext[i].cruiseRate = 0;
			mBlocksNext[i].cruiseInterval = 0;
			mBlocksNext[i].cruiseReload = 0;
		}
	}

	// exit if no movements
	if (!nothingToDo)
	{

		//***************************************************************
		// Prepare for motor synchronization.
		//
		// Find the motor with the longest movement (distance * speed) and
		// adjust the speed of the other motors to match duration.
		//
		// Use bubble sort to organize longest duration first.
		//
		for (i = ISRA; i < NUM_MOTORS; i++)
		{
			_sort[i] = i; 	// default ISR assignment motor 0,1,2
		}

		//
		// sort motor movements by their respective durations, longest first
		//
		do {
			swap = false;
			for (i = MOTOR_0; i < NUM_MOTORS - 1; i++) {
				if (mBlocksNext[_sort[i]].totalTime < mBlocksNext[_sort[i + 1]].totalTime)
				{
					uint16_t b = _sort[i];
					_sort[i] = _sort[i + 1];
					_sort[i + 1] = b;
					swap = true;
				}
			}
		} while (swap);

		//***************************************************************
		// Calculate accel / decel profiles
		//
		// If mSync (synchronize motor movements) is true,
		// modify speeds of shorter duration movements to match motor with
		// longest duration.
		//
		for (i = ISRA; i < NUM_MOTORS; i++)
		{
			if (mBlocksNext[_sort[i]].numSteps > 0)
			{
				//
				// motor needs acceleration profile if acceleration > 0 and
				// target cruise speed is fast enough to need it.
				//
				if(mBlocksNext[_sort[i]].acceleration > 0.0 && mBlocksNext[_sort[i]].cruiseRate >= MOTOR_START_SPEED)
				{
					//
					// calc interval of first step based on acceleration
					//
					mBlocksNext[_sort[i]].startInterval = ONE_MILLION / sqrt(2.0 * mBlocksNext[_sort[i]].acceleration);
					NOTMORE(mBlocksNext[_sort[i]].startInterval, MAX_ACCEL_INTERVAL);  // accel upper bound

					//
					// calc total duration including accel / decel slopes
					//
					mBlocksNext[_sort[i]].totalTime = 0;
					mBlocksNext[_sort[i]].rampTime = 0;
					startT = uint32_t(mBlocksNext[_sort[i]].startInterval);
					endT = uint32_t(mBlocksNext[_sort[i]].cruiseInterval);
					sTot = 0;
					tTot = 0;

					// iterative loop to get sum of ramp intervals and
					// number of steps from start to cruise speed.
					// Brute force yes, but it works ;>)
					//
					for(rampSteps=0; rampSteps<LOOKUP_TBL_SIZE; rampSteps++)
					{
						//
						// need to accumulate sum of ramp intervals to calc new
						// speed on-the-fly
						//
						CALC_ACCEL(pun.val32, mBlocksNext[_sort[i]].startInterval, rampSteps+1);
						startT = pun.val16[1];

						if(startT <= endT)	// all done if we have reached the desired cruising speed
							break;

						sTot += startT;
						//
						// stretch shorter duration movements to match longest
						// duration. Cruise speed constantly tweaked while building the ramp.
						//
						if(i != ISRA && mSync)  // only motors with shorter durations allowed in here
						{
							//
							// calc duration of cruise steps ---
							//
							tTot = uint32_t(mBlocksNext[_sort[i]].cruiseSteps) - ((rampSteps+1) * 2);
							tTot *= (endT);
							//
							// --- plus accumulated ramp time
							//
							tTot += (sTot * 2);
							//
							// compare ratio of current duration to longest
							//
							fr = float(mBlocksNext[_sort[ISRA]].totalTime) / float(tTot);
							//
							// keep tweaking until...
							//
							endT *= fr;
						}
						//
						// If movement is short it may never reach cruise speed
						// resulting in a triangular trajectory.
						//
						if(((rampSteps+1) * 2) >= mBlocksNext[_sort[i]].cruiseSteps)
							break;
					}

					//
					// new cruise speed in 'endT'
					//
					mBlocksNext[_sort[i]].cruiseInterval = endT;
					mBlocksNext[_sort[i]].cruiseReload = endT;
					mBlocksNext[_sort[i]].cruiseRate = (1.0 / endT) * ONE_MILLION;

					//
					// If new speed is slow and doesn't need accel, kill
					// accel profile and go with constant speed (square) profile
					//
					if(mBlocksNext[_sort[i]].cruiseRate < MOTOR_START_SPEED)
					{
						mBlocksNext[_sort[i]].rampSteps = 0;
						fr = float(mBlocksNext[_sort[ISRA]].totalTime) /
								float(mBlocksNext[_sort[i]].numSteps);
						mBlocksNext[_sort[i]].cruiseInterval = round(fr);
						mBlocksNext[_sort[i]].cruiseReload = mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].startInterval = mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].totalTime = mBlocksNext[_sort[i]].numSteps * mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].cruiseRate = (1.0 / float(mBlocksNext[_sort[i]].cruiseInterval)) * ONE_MILLION;
					}
					else    // still have accel - deal with ramp & cruise steps
					{
						//
						// handle odd numbered steps
						//
						if((rampSteps << 1) > mBlocksNext[_sort[i]].cruiseSteps)
							rampSteps--;
						mBlocksNext[_sort[i]].rampSteps	= rampSteps;
						mBlocksNext[_sort[i]].cruiseSteps -= (rampSteps * 2);
						mBlocksNext[_sort[i]].rampTime = sTot;
						mBlocksNext[_sort[i]].totalTime = mBlocksNext[_sort[i]].rampTime * 2;	// accel + decel
						mBlocksNext[_sort[i]].totalTime += (uint32_t(mBlocksNext[_sort[i]].cruiseSteps) *
								uint32_t(mBlocksNext[_sort[i]].cruiseInterval));
					}
				}
				//
				// constant cruise speed here
				//
				else
				{
					mBlocksNext[_sort[i]].rampSteps = 0;
					//
					// sync movement durations here
					//
					if(mSync && (i != ISRA))
					{
						//
						// compare ratio of longest duration to this duration
						//
						fr = float(mBlocksNext[_sort[ISRA]].totalTime) /
								float(mBlocksNext[_sort[i]].numSteps);
						//
						// apply ratio to this movement
						//
						mBlocksNext[_sort[i]].cruiseInterval = round(fr);
						mBlocksNext[_sort[i]].cruiseReload = mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].startInterval = mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].totalTime = mBlocksNext[_sort[i]].numSteps * mBlocksNext[_sort[i]].cruiseInterval;
						mBlocksNext[_sort[i]].cruiseRate = (1.0 / float(mBlocksNext[_sort[i]].cruiseInterval)) * ONE_MILLION;
					}
				}
			}
		}
	}
	else
		_planResult = ERR_PLAN_VOID;

	_planValid = true;
	return _planResult;	// all is OK
}

//###################################################################
// runMotors() - The main attraction!!!
//
// Motor movement planning is done in planMoves().
//
// Motor driver should have been enabled via 'setEnable()' prior to
// calling here.
//
//
//###################################################################
//
uint8_t jStepper::runMotors(float pos0, float pos1, float pos2, bool mSync)
{
	// uint8_t errcode = ERR_NONE;
	if(isRunning(MOTOR_ALL))		// exit if any motors are still running
		return ERR_MOTORS_RUNNING;

	//
	// if plan has not already been called we will do it here
	//
	if(!_planValid)
		planMoves(pos0, pos1, pos2, mSync);
	memcpy(&mBlocks, &mBlocksNext, sizeof(mBlocks));		// execute previous plan
	_planValid = false;						// reset auto planning

	if(_planResult == ERR_PLAN_VOID)		// nothing to do...just return happy!
		return ERR_NONE;

	if(_planResult != ERR_NONE)				// planner error found!
		return _planResult;


	//***************************************************************
	// Set up timer regs and compare interrupts.
	//
	//cli();
	TCCRA = 0x00; 		// stop the timer
	TCCRB = 0x00;  	
	TIMSK = 0x00; 

	// Load timer compare regs with the starting interval
	// An interrupt from the compare match will generate step pulses
	// and calc subsequent intervals.
	if(mBlocks[MOTOR_0].numSteps > 0)
	{
		//
		// prepare motor direction and ISR state machine for this motor
		//
		if (mBlocks[MOTOR_0].newPosition >= mBlocks[MOTOR_0].curPosition)
			setDirection(MOTOR_0, MOTOR_DIRECTION_IN);
		else
			setDirection(MOTOR_0, MOTOR_DIRECTION_OUT);

		mBlocks[MOTOR_0].curPosition = mBlocks[MOTOR_0].newPosition; // can update cur position now
		mBlocks[MOTOR_0].mAction = (mBlocks[MOTOR_0].rampSteps > 0) ? STEP_ACCEL : STEP_CRUISE;
		mBlocks[MOTOR_0].rampStepCount = 0;

		// check if step pulse duration is longer than one timer cycle
		if(mBlocks[MOTOR_0].cruiseInterval > 32767)
		{
			mBlocks[MOTOR_0].mAction = STEP_LONG;  // set mode for long step intervals
			OCRA = ((mBlocks[MOTOR_0].cruiseInterval & 0x7FFF) << 1);  // interrupt delay in usec *2
		}
		else
			OCRA = (mBlocks[MOTOR_0].startInterval << 1);    // interrupt delay in usec *2
	  
		SETb(TIMSK, OCIEA);		// enable compare A interrupt
		SETb(TIFR, OCFA);   	// clear any pending interrupts
	}
   
	if(mBlocks[MOTOR_1].numSteps > 0)
	{
		//
		// prepare motor direction and ISR state machine for this motor
		//
		if (mBlocks[MOTOR_1].newPosition >= mBlocks[MOTOR_1].curPosition)
			setDirection(MOTOR_1, MOTOR_DIRECTION_IN);
		else
			setDirection(MOTOR_1, MOTOR_DIRECTION_OUT);

		mBlocks[MOTOR_1].curPosition = mBlocks[MOTOR_1].newPosition; // can update cur position now
		mBlocks[MOTOR_1].mAction = (mBlocks[MOTOR_1].rampSteps > 0) ? STEP_ACCEL : STEP_CRUISE;
		mBlocks[MOTOR_1].rampStepCount = 0;

		if(mBlocks[MOTOR_1].cruiseInterval > 32767)
		{
			mBlocks[MOTOR_1].mAction = STEP_LONG;  // set mode for long step intervals
			OCRB = ((mBlocks[MOTOR_1].cruiseInterval & 0x7FFF) << 1);
		}
		else
			OCRB = (mBlocks[MOTOR_1].startInterval << 1);    // interrupt delay in usec *2

		SETb(TIMSK, OCIEB);		// enable compare B interrupt
		SETb(TIFR, OCFB);   	// clear any pending interrupts
	}
   
	if(mBlocks[MOTOR_2].numSteps > 0)
	{
		//
		// prepare motor direction and ISR state machine for this motor
		//
		if (mBlocks[MOTOR_2].newPosition >= mBlocks[MOTOR_2].curPosition)
			setDirection(MOTOR_2, MOTOR_DIRECTION_IN);
		else
			setDirection(MOTOR_2, MOTOR_DIRECTION_OUT);

		mBlocks[MOTOR_2].curPosition = mBlocks[MOTOR_2].newPosition; // can update cur position now
		mBlocks[MOTOR_2].mAction = (mBlocks[MOTOR_2].rampSteps > 0) ? STEP_ACCEL : STEP_CRUISE;
		mBlocks[MOTOR_2].rampStepCount = 0;

		if(mBlocks[MOTOR_2].cruiseInterval > 32767)
		{
			mBlocks[MOTOR_2].mAction = STEP_LONG;  // special mode for long step intervals
			OCRC = ((mBlocks[MOTOR_2].cruiseInterval & 0x7FFF) << 1);
		}
		else
			OCRC = (mBlocks[MOTOR_2].startInterval << 1);    // interrupt delay in usec *2

		SETb(TIMSK, OCIEC);		// enable compare C interrupt
		SETb(TIFR, OCFC);   	// clear any pending interrupts
	}

	// start the selected timer in free running (normal) mode with 2Mhz clock
	TCNT = 0x00;
	TCCRB = 0x02;
	//sei();
	return ERR_NONE;
}


//*******************************************************************
//
void jStepper::addTimerCallBack(uint8_t whichTimerInt, void *p)
{
	isrRedirectArray[whichTimerInt] = (functPtr)p;
}


//*******************************************************************
//
void jStepper::isrRedirect(uint8_t whichTimerInt)
{
	if(isrRedirectArray[whichTimerInt] != NULL)		// ptr NULL?
		(*(isrRedirectArray[whichTimerInt]))();		// if not call external isr handler
}


//*******************************************************************
//
void jStepper::setStepsPerUnit(uint16_t su0, uint16_t su1, uint16_t su2)
{
	mBlocks[MOTOR_0].stepsPerUnit = su0;
	mBlocks[MOTOR_1].stepsPerUnit = su1;
	mBlocks[MOTOR_2].stepsPerUnit = su2;
}


//*******************************************************************
//
uint16_t jStepper::getStepsPerUnit(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].stepsPerUnit;
	else
		return 0;
}


//*******************************************************************
//
uint8_t jStepper::getLastResult(uint8_t motorNum)
{
	if(motorNum < NUM_MOTORS)
		return mBlocks[motorNum].lastResult;
	else
		return ERR_INVALID_MOTOR;
}


//###################################################################
// Interrupt vectors for all timers
//###################################################################

//*******************************************************************
//
ISR(TIMER1_COMPA_vect)
{
	if(psPtr_1)
		psPtr_1->timerISRA();	// call instance member function
	else
		isrRedirectObj->isrRedirect(TMR_1_CMPA);	// else call user installed function
}


//*******************************************************************
// 
ISR(TIMER1_COMPB_vect)
{
	if(psPtr_1)
		psPtr_1->timerISRB();
	else
		isrRedirectObj->isrRedirect(TMR_1_CMPB);
}


//*******************************************************************
// 
ISR(TIMER1_COMPC_vect)
{
	if(psPtr_1)
		psPtr_1->timerISRC();
	else
		isrRedirectObj->isrRedirect(TMR_1_CMPC);
}


//*******************************************************************
//
ISR(TIMER1_OVF_vect)
{
	if(psPtr_1)
		psPtr_1->homeISR();
	else
		isrRedirectObj->isrRedirect(TMR_1_OVF);
}


//*******************************************************************
//
ISR(TIMER3_COMPA_vect)
{
	if(psPtr_3)
		psPtr_3->timerISRA();
	else
		isrRedirectObj->isrRedirect(TMR_3_CMPA);
}


//*******************************************************************
// 
ISR(TIMER3_COMPB_vect)
{
	if(psPtr_3)
		psPtr_3->timerISRB();
	else
		isrRedirectObj->isrRedirect(TMR_3_CMPB);
}


//*******************************************************************
// 
ISR(TIMER3_COMPC_vect)
{
	if(psPtr_3)
		psPtr_3->timerISRC();
	else
		isrRedirectObj->isrRedirect(TMR_3_CMPC);
}


//*******************************************************************
//
ISR(TIMER3_OVF_vect)
{
	if(psPtr_3)
		psPtr_3->homeISR();
	else
		isrRedirectObj->isrRedirect(TMR_3_OVF);
}


//*******************************************************************
//
ISR(TIMER4_COMPA_vect)
{
	if(psPtr_4)
		psPtr_4->timerISRA();
	else
		isrRedirectObj->isrRedirect(TMR_4_CMPA);
}


//*******************************************************************
// 
ISR(TIMER4_COMPB_vect)
{
	if(psPtr_4)
		psPtr_4->timerISRB();
	else
		isrRedirectObj->isrRedirect(TMR_4_CMPB);
}


//*******************************************************************
// 
ISR(TIMER4_COMPC_vect)
{
	if(psPtr_4)
		psPtr_4->timerISRC();
	else
		isrRedirectObj->isrRedirect(TMR_4_CMPC);
}


//*******************************************************************
//
ISR(TIMER4_OVF_vect)
{
	if(psPtr_4)
		psPtr_4->homeISR();
	else
		isrRedirectObj->isrRedirect(TMR_4_OVF);
}


//*******************************************************************
//
ISR(TIMER5_COMPA_vect)
{
	if(psPtr_5)
		psPtr_5->timerISRA();
	else
		isrRedirectObj->isrRedirect(TMR_5_CMPA);
}


//*******************************************************************
// 
ISR(TIMER5_COMPB_vect)
{
	if(psPtr_5)
		psPtr_5->timerISRB();
	else
		isrRedirectObj->isrRedirect(TMR_5_CMPB);
}


//*******************************************************************
// 
ISR(TIMER5_COMPC_vect)
{
	if(psPtr_5)
		psPtr_5->timerISRC();
	else
		isrRedirectObj->isrRedirect(TMR_5_CMPC);
}


//*******************************************************************
//
ISR(TIMER5_OVF_vect)
{
	if(psPtr_5)
		psPtr_5->homeISR();
	else
		isrRedirectObj->isrRedirect(TMR_5_OVF);
}


