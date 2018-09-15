/*
 *  jStepper.cpp
 */

#include "jStepper.h"


//###############################################################
// made these macros to inline code and avoid overhead
// of a function call
//
#define STEP_MOTOR_0 \
		WRITE_NC(MOTOR_0_STEP_PIN, STEP_PULSE_ASSERT); \
		DELAY_1_NOP;  \
		WRITE_NC(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);

#define STEP_MOTOR_1 \
		WRITE_NC(MOTOR_1_STEP_PIN, STEP_PULSE_ASSERT); \
		DELAY_1_NOP; \
		WRITE_NC(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);

#define STEP_MOTOR_2 \
		WRITE_NC(MOTOR_2_STEP_PIN, STEP_PULSE_ASSERT); \
		DELAY_1_NOP; \
		WRITE_NC(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);


#define STEP_MOTOR(MOTNUM) do{ \
	switch (MOTNUM) { \
	case MOTOR_0: STEP_MOTOR_0 \
		break; \
	case MOTOR_1: STEP_MOTOR_1 \
		break; \
	case MOTOR_2: STEP_MOTOR_2 \
		break; \
	} }while(0)

    jStepper *psPtr;    // global jStepper obj for ISR's

//###############################################################
//
// jStepper constructor
jStepper::jStepper(void)
{
	uint16_t i;
	// _extIOfunc = extIOfunc;  // if using external function for???

	// set default parameters for all motors
	for (i = MOTOR_0; i < NUM_MOTORS; i++)
	{
		mBlocks[i].minPosition = 0.0;
		mBlocks[i].maxPosition = 0.0;
		mBlocks[i].curPosition = 0.0;
		mBlocks[i].newPosition = 0.0;
		mBlocks[i].acceleration = 0.0;
		mBlocks[i].speed = 10.0;         // something conservative
		mBlocks[i].maxSpeed = MAX_SPEED; // 
		mBlocks[i].cruiseSteps = 0;
		mBlocks[i].numSteps = 0;
		mBlocks[i].positionKnown = false;
		mBlocks[i].mAction = STEP_DONE;

		switch(i)
		{
			case MOTOR_0:
				mBlocks[0].stepsPerMM = MOTOR_0_STEPS_PER_MM;
				break;
			case MOTOR_1:
				mBlocks[1].stepsPerMM = MOTOR_1_STEPS_PER_MM;
				break;
			case MOTOR_2:
				mBlocks[2].stepsPerMM = MOTOR_2_STEPS_PER_MM;
				break;
		}
	}

//*******************************************************************
// initialize I/O pins
//
// output pins ---
//
	SET_OUTPUT(MOTOR_0_STEP_PIN);
	SET_OUTPUT(MOTOR_1_STEP_PIN);
	SET_OUTPUT(MOTOR_2_STEP_PIN);

	SET_OUTPUT(MOTOR_0_DIR_PIN);
	SET_OUTPUT(MOTOR_1_DIR_PIN);
	SET_OUTPUT(MOTOR_2_DIR_PIN);

	SET_OUTPUT(MOTOR_0_ENB_PIN);
	SET_OUTPUT(MOTOR_1_ENB_PIN);
	SET_OUTPUT(MOTOR_2_ENB_PIN);

//
// input pins
//
	SET_INPUT_PULLUP(ENDSTOP_MIN_0_PIN);	// set up endstop input w/pullup
	SET_INPUT_PULLUP(ENDSTOP_MIN_1_PIN);
	SET_INPUT_PULLUP(ENDSTOP_MIN_2_PIN);
	

#if defined(USE_MAX_ENDSTOPS)
	SET_INPUT_PULLUP(ENDSTOP_MAX_0_PIN);	// set up endstop input w/pullup
	SET_INPUT_PULLUP(ENDSTOP_MAX_1_PIN);
	SET_INPUT_PULLUP(ENDSTOP_MAX_2_PIN);
#endif

	setEnabled(false, false, false);	// disable all drivers

	WRITE(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);		// deassert step pin
	WRITE(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);
	WRITE(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);

//
// initialize signal states
//


	psPtr = this;       // initialize ptr to jStepper object
}


//###############################################################
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


//###############################################################
// getAcceleration()
//
float jStepper::getAcceleration(uint8_t motorNum)
{
	switch(motorNum)
	{
		case MOTOR_0:
			return mBlocks[MOTOR_0].acceleration;
			break;
         
		case MOTOR_1:
			return mBlocks[MOTOR_1].acceleration;
			break;

		case MOTOR_2:
			return mBlocks[MOTOR_2].acceleration;
			break;     
			
		default:
			return -1;
			break;
	}
}

//###############################################################
//
// set motor speed in mm / sec.
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


//###############################################################
//
float jStepper::getSpeed(uint8_t motorNum)
{
   switch(motorNum)
   {
      case MOTOR_0:
         return mBlocks[MOTOR_0].speed;
         break;
         
      case MOTOR_1:
         return mBlocks[MOTOR_1].speed;
         break;

      case MOTOR_2:
         return mBlocks[MOTOR_2].speed;
         break;    
         
      default:
    	  return -1;
    	  break;
   }
}


//###############################################################
//
// set motor speed in mm / sec.
void jStepper::setMaxSpeed(uint8_t motorNum, float mMaxSpeed)   // speed in mm/sec
{
   mMaxSpeed = abs(mMaxSpeed);   // only positive number
   
   if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
   {
      mBlocks[MOTOR_0].maxSpeed = mMaxSpeed;
   }

   if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
   {
      mBlocks[MOTOR_1].maxSpeed = mMaxSpeed;
   }
   
   if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
   {
      mBlocks[MOTOR_2].maxSpeed = mMaxSpeed;
   }   
}


//###############################################################
//
float jStepper::getMaxSpeed(uint8_t motorNum)
{
   switch(motorNum)
   {
      case MOTOR_0:
         return mBlocks[MOTOR_0].maxSpeed;
         break;
         
      case MOTOR_1:
         return mBlocks[MOTOR_1].maxSpeed;
         break;

      case MOTOR_2:
         return mBlocks[MOTOR_2].maxSpeed;
         break;  
         
      default:
    	  return -1;
    	  break;
   }
}


//###############################################################
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


//###############################################################
//
void jStepper::quickStop(uint8_t motorNum) 
{
   if(motorNum == MOTOR_0 || motorNum == MOTOR_ALL)
   {
      WRITE(MOTOR_0_ENB_PIN, !MOTOR_ENABLE_LEVEL);
      mBlocks[MOTOR_0].mAction = STEP_DONE;
   }

   if(motorNum == MOTOR_1 || motorNum == MOTOR_ALL)
   {
      WRITE(MOTOR_1_ENB_PIN, !MOTOR_ENABLE_LEVEL);
      mBlocks[MOTOR_1].mAction = STEP_DONE;
   }

   if(motorNum == MOTOR_2 || motorNum == MOTOR_ALL)
   {
      WRITE(MOTOR_2_ENB_PIN, !MOTOR_ENABLE_LEVEL);
      mBlocks[MOTOR_2].mAction = STEP_DONE;
   }
}


//###############################################################
//
uint8_t jStepper::setPositionAbs(float pos0, float pos1, float pos2) 
{
	uint8_t ecode = ERR_NONE;
	
    if(pos0 < mBlocks[MOTOR_0].minPosition || pos0 > mBlocks[MOTOR_0].maxPosition)
    	ecode = ERR_OUTSIDE_BOUNDARY;
    else 
    	mBlocks[MOTOR_0].newPosition = pos0;
    
    if(pos1 < mBlocks[MOTOR_1].minPosition || pos1 > mBlocks[MOTOR_1].maxPosition)
    	ecode = ERR_OUTSIDE_BOUNDARY;
    else 
    	mBlocks[MOTOR_1].newPosition = pos1;   
    
    if(pos2 < mBlocks[MOTOR_2].minPosition || pos2 > mBlocks[MOTOR_2].maxPosition)
    	ecode = ERR_OUTSIDE_BOUNDARY;
    else 
    	mBlocks[MOTOR_2].newPosition = pos2;    
    
    return ecode;
}


//###############################################################
//
float jStepper::getPositionAbs(uint8_t motorNum) 
{
	if(motorNum >= NUM_MOTORS)
		return -1;
	else
		return mBlocks[motorNum].curPosition;
}


//###############################################################
// setMinPosition() sets min position for the given motor
//
void jStepper::setMinPosition(float minPos0, float minPos1, float minPos2) 
{
    mBlocks[MOTOR_0].minPosition = minPos0;
    mBlocks[MOTOR_1].minPosition = minPos1;
    mBlocks[MOTOR_2].minPosition = minPos2;    
}
  

//###############################################################
// returns 0 if error
float jStepper::getMinPosition(uint8_t motorNum)
{
	if(motorNum >= NUM_MOTORS)
		return -1;
	else
		return mBlocks[motorNum].minPosition;
}


//###############################################################
// setMaxPosition() sets max position for the given motor
//
void jStepper::setMaxPosition(float maxPos0, float maxPos1, float maxPos2) 
{
    mBlocks[MOTOR_0].maxPosition = maxPos0;
    mBlocks[MOTOR_1].maxPosition = maxPos1;    
    mBlocks[MOTOR_2].maxPosition = maxPos2;   
}


//###############################################################
// returns -1 if error
//
float jStepper::getMaxPosition(uint8_t motorNum)
{
	if(motorNum >= NUM_MOTORS)
		return -1;
	else
		return mBlocks[motorNum].maxPosition;
}


//###############################################################
//
void jStepper::setDirection(uint8_t motorNum, uint8_t dir)
{
	switch(motorNum)
	{
		case MOTOR_0:
			if (dir == MOTOR_DIRECTION_IN)
				WRITE(MOTOR_0_DIR_PIN, MOTOR_0_DIRECTION_IN);
			else
				WRITE(MOTOR_0_DIR_PIN, !MOTOR_0_DIRECTION_IN);
			break;
		
		case MOTOR_1:
			if (dir == MOTOR_DIRECTION_IN)
				WRITE(MOTOR_1_DIR_PIN, MOTOR_1_DIRECTION_IN);
			else
				WRITE(MOTOR_1_DIR_PIN, !MOTOR_1_DIRECTION_IN);
			break;
			
		case MOTOR_2:
			if (dir == MOTOR_DIRECTION_IN)
				WRITE(MOTOR_2_DIR_PIN, MOTOR_2_DIRECTION_IN);
			else
				WRITE(MOTOR_2_DIR_PIN, !MOTOR_2_DIRECTION_IN);
			break;			
	}
}


//###############################################################
//
uint8_t jStepper::getDirection(uint8_t motorNum)
{
	switch (motorNum) 
    {
	    case MOTOR_0:
            if(READ(MOTOR_0_DIR_PIN) == MOTOR_0_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
            break;

	    case MOTOR_1:
            if(READ(MOTOR_1_DIR_PIN) == MOTOR_1_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
		    break;

	    case MOTOR_2:
            if(READ(MOTOR_2_DIR_PIN) == MOTOR_2_DIRECTION_IN)
        		return MOTOR_DIRECTION_IN;
            else
                return MOTOR_DIRECTION_OUT;
		    break;
		    
	    default:
	    	return 0;
	    	break;
	}
}


//###############################################################
//
void jStepper::setEnabled(bool enab0, bool enab1, bool enab2)
{
    if (enab0)
    	WRITE(MOTOR_0_ENB_PIN, MOTOR_ENABLE_LEVEL);
	else
	    WRITE(MOTOR_0_ENB_PIN, !MOTOR_ENABLE_LEVEL);

    if (enab1)
    	WRITE(MOTOR_1_ENB_PIN, MOTOR_ENABLE_LEVEL);
	else
	    WRITE(MOTOR_1_ENB_PIN, !MOTOR_ENABLE_LEVEL);

    if (enab2)
    	WRITE(MOTOR_2_ENB_PIN, MOTOR_ENABLE_LEVEL);
	else
	    WRITE(MOTOR_2_ENB_PIN, !MOTOR_ENABLE_LEVEL);

}


//###############################################################
//
bool jStepper::isEnabled(uint8_t motorNum)
{
    switch(motorNum)
    {
        case MOTOR_0:
            return (READ(MOTOR_0_ENB_PIN) == MOTOR_ENABLE_LEVEL);
            break;

        case MOTOR_1:
           	return (READ(MOTOR_1_ENB_PIN) == MOTOR_ENABLE_LEVEL);
            break;

        case MOTOR_2:
           	return (READ(MOTOR_2_ENB_PIN) == MOTOR_ENABLE_LEVEL);
            break;
    }
    return false;
}


//###############################################################
// move motor one step towards setDirection()
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


//###############################################################
// returns true if endstop detected
//
bool jStepper::atMinEndStop(uint8_t motorNum) {
	switch (motorNum) 
    {
	    case MOTOR_0:
	    	return (READ(ENDSTOP_MIN_0_PIN) == IN_ENDSTOP);
		    break;

	    case MOTOR_1:
	    	return (READ(ENDSTOP_MIN_1_PIN) == IN_ENDSTOP);
		    break;

	    case MOTOR_2:
	    	return (READ(ENDSTOP_MIN_2_PIN) == IN_ENDSTOP);
		    break;
	}
	return false;
}


//###############################################################
// returns true if endstop detected
//
bool jStepper::atMaxEndStop(uint8_t motorNum) {
	switch (motorNum) 
    {
	    case MOTOR_0:
    		return (READ(ENDSTOP_MAX_0_PIN) == IN_ENDSTOP);
		    break;

	    case MOTOR_1:
	    	return (READ(ENDSTOP_MAX_1_PIN) == IN_ENDSTOP);
		    break;

	    case MOTOR_2:
	    	return (READ(ENDSTOP_MAX_2_PIN) == IN_ENDSTOP);
		    break;
	}
	return false;
}


//###############################################################
//
void jStepper::setPositionKnown(uint8_t motorNum, bool known)
{
    if(motorNum < NUM_MOTORS)
    	mBlocks[motorNum].positionKnown = known;
}


//###############################################################
//
uint8_t jStepper::isPositionKnown(uint8_t motorNum)
{
    if(motorNum >= NUM_MOTORS)
        return false;

    return mBlocks[motorNum].positionKnown;
}


//###############################################################
//
void jStepper::timerISRA(void)
{
	static union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
	
	switch (mBlocks[MOTOR_0].mAction)     // check profile state
	{
    	case STEP_CRUISE:			// constant speed
    		WRITE_NC(MOTOR_0_STEP_PIN, STEP_PULSE_ASSERT);		// step pulse assert
    		OCRA += uint16_t(mBlocks[MOTOR_0].cruiseInterval << 1);
			if (--mBlocks[MOTOR_0].cruiseSteps == 0)
			{
			   if(mBlocks[MOTOR_0].rampSteps)
				   mBlocks[MOTOR_0].mAction = STEP_DECEL;
			   else
			   {
				   CBI(TIMSK, OCIEA);  // all done 
				   mBlocks[MOTOR_0].mAction = STEP_DONE;
			   }
			}		
			WRITE_NC(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);		// step pulse deassert
			break;

		case STEP_ACCEL:		// accelerating
			WRITE_NC(MOTOR_0_STEP_PIN, STEP_PULSE_ASSERT);		
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_0].startInterval, ++mBlocks[MOTOR_0].rampStepCount);
			OCRA += pun.val16[1] << 1;
			if (mBlocks[MOTOR_0].rampStepCount >= mBlocks[MOTOR_0].rampSteps)
			   mBlocks[MOTOR_0].mAction = (mBlocks[MOTOR_0].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			WRITE_NC(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			WRITE_NC(MOTOR_0_STEP_PIN, STEP_PULSE_ASSERT);	
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_0].startInterval, mBlocks[MOTOR_0].rampStepCount--);
			OCRA += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_0].rampStepCount)
			{
			   CBI(TIMSK, OCIEA);  // last phase, all done!
			   mBlocks[MOTOR_0].mAction = STEP_DONE;
			}
			WRITE_NC(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);			
			break;
			
      case STEP_LONG:			// long interval
         if(mBlocks[MOTOR_0].cruiseInterval < 32768)
         {
        	WRITE_NC(MOTOR_0_STEP_PIN, STEP_PULSE_ASSERT);
            mBlocks[MOTOR_0].cruiseInterval = mBlocks[MOTOR_0].cruiseReload;
            frac = mBlocks[MOTOR_0].cruiseInterval & 0x7FFF;
            NOTLESS(frac, 16);	// move fwd enough so we don't overrun timer
            OCRA = OCRA + (frac << 1);  // move fractional part of interval ahead
			if (--mBlocks[MOTOR_0].cruiseSteps == 0)
			{
  		       mBlocks[MOTOR_0].mAction = STEP_DONE;
               CBI(TIMSK, OCIEA);  // disable int's   		         
            }
            else
               SBI(TIFR, OCFA);    	// clear any pending interrupts		
			
			WRITE_NC(MOTOR_0_STEP_PIN, !STEP_PULSE_ASSERT);  			
         }
         else
            mBlocks[MOTOR_0].cruiseInterval -= 32768;    // interval minus whole number

         break;			
	}   // end switch()
}


//###############################################################
//
void jStepper::timerISRB(void)
{
	static union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
		
   switch (mBlocks[MOTOR_1].mAction)     // check profile state
   {
      case STEP_CRUISE:			// constant speed
    	  WRITE_NC(MOTOR_1_STEP_PIN, STEP_PULSE_ASSERT);       
    	  OCRB += (mBlocks[MOTOR_1].cruiseInterval << 1);
    	  if (--mBlocks[MOTOR_1].cruiseSteps == 0)
    	  {
    		  if(mBlocks[MOTOR_1].rampSteps)
    			  mBlocks[MOTOR_1].mAction = STEP_DECEL;
    		  else
    		  {
    			  CBI(TIMSK, OCIEB);  // all done 
    			  mBlocks[MOTOR_1].mAction = STEP_DONE;
    		  }
    	  }	
    	  WRITE_NC(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);
    	  break;

		case STEP_ACCEL:		// accelerating
			WRITE_NC(MOTOR_1_STEP_PIN, STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_1].startInterval, ++mBlocks[MOTOR_1].rampStepCount);
			OCRB += pun.val16[1] << 1;
			if (mBlocks[MOTOR_1].rampStepCount >= mBlocks[MOTOR_1].rampSteps)
			   mBlocks[MOTOR_1].mAction = (mBlocks[MOTOR_1].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			
			WRITE_NC(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			WRITE_NC(MOTOR_1_STEP_PIN, STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_1].startInterval, mBlocks[MOTOR_1].rampStepCount--);
			OCRB += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_1].rampStepCount)
			{
			   CBI(TIMSK, OCIEB);  // last phase, all done!
			   mBlocks[MOTOR_1].mAction = STEP_DONE;
			}
			WRITE_NC(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);
			break;
			
      case STEP_LONG:			// long interval
         if(mBlocks[MOTOR_1].cruiseInterval < 32768)
         {
        	 WRITE_NC(MOTOR_1_STEP_PIN, STEP_PULSE_ASSERT);
        	 mBlocks[MOTOR_1].cruiseInterval = mBlocks[MOTOR_1].cruiseReload;
        	 frac = mBlocks[MOTOR_1].cruiseInterval & 0x7FFF;
        	 NOTLESS(frac, 16);
        	 OCRB = OCRB + (frac << 1);  
			 	 if (--mBlocks[MOTOR_1].cruiseSteps == 0)
			 	 {
			 		 mBlocks[MOTOR_1].mAction = STEP_DONE;
			 		 CBI(TIMSK, OCIEB);  // disable int's   		         
			 	 }
			 	 else
			 		 SBI(TIFR, OCFB);    	// clear pending interrupts		
			 WRITE_NC(MOTOR_1_STEP_PIN, !STEP_PULSE_ASSERT);
         }
         else
            mBlocks[MOTOR_1].cruiseInterval -= 32768;    // interval - whole number
         break;				
	}   // end switch()
}


//###############################################################
//
void jStepper::timerISRC(void)
{
	static union {
		uint16_t val16[2];
		uint32_t val32;
	} pun;
	uint16_t frac;
		
   switch (mBlocks[MOTOR_2].mAction)     // check profile state
   {
      case STEP_CRUISE:			// constant speed
    	  WRITE_NC(MOTOR_2_STEP_PIN, STEP_PULSE_ASSERT);
    	  OCRC += (mBlocks[MOTOR_2].cruiseInterval << 1);
    	  if (--mBlocks[MOTOR_2].cruiseSteps == 0)
    	  {
    		  if(mBlocks[MOTOR_2].rampSteps)
    			  mBlocks[MOTOR_2].mAction = STEP_DECEL;
    		  else
    		  {
    			  CBI(TIMSK, OCIEC);  // all done 
     		      mBlocks[MOTOR_2].mAction = STEP_DONE;  
    		  }
    	  }	
    	  WRITE_NC(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);
    	  break;

		case STEP_ACCEL:		// accelerating
			WRITE_NC(MOTOR_2_STEP_PIN, STEP_PULSE_ASSERT);;
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_2].startInterval, ++mBlocks[MOTOR_2].rampStepCount);
			OCRC += pun.val16[1] << 1;
			if (mBlocks[MOTOR_2].rampStepCount >= mBlocks[MOTOR_2].rampSteps)
			   mBlocks[MOTOR_2].mAction = (mBlocks[MOTOR_2].cruiseSteps > 0) ? STEP_CRUISE : STEP_DECEL; // decel if no cruise steps
			
			WRITE_NC(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);
			break;

		case STEP_DECEL:		// decelerating
			WRITE_NC(MOTOR_2_STEP_PIN, STEP_PULSE_ASSERT);
			CALC_ACCEL(pun.val32, mBlocks[MOTOR_2].startInterval, mBlocks[MOTOR_2].rampStepCount--);
			OCRC += pun.val16[1] << 1;
			if (!mBlocks[MOTOR_2].rampStepCount)
			{
			   CBI(TIMSK, OCIEC);  // last phase, all done!
			   mBlocks[MOTOR_2].mAction = STEP_DONE;
			}
			WRITE_NC(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);			
			break;
			
		case STEP_LONG:			// long interval
			if(mBlocks[MOTOR_2].cruiseInterval < 32768)
			{
				WRITE_NC(MOTOR_2_STEP_PIN, STEP_PULSE_ASSERT);
				mBlocks[MOTOR_2].cruiseInterval = mBlocks[MOTOR_2].cruiseReload;
				frac = mBlocks[MOTOR_2].cruiseInterval & 0x7FFF;
				NOTLESS(frac, 16);
				OCRC = OCRC + (frac << 1);  // move fractional part of interval ahead
				if (--mBlocks[MOTOR_2].cruiseSteps == 0)
				{
					mBlocks[MOTOR_2].mAction = STEP_DONE;
					CBI(TIMSK, OCIEC);  // disable int's   		         
				}
				else
					SBI(TIFR, OCFC);    	// clear pending interrupts		
				WRITE_NC(MOTOR_2_STEP_PIN, !STEP_PULSE_ASSERT);
			}
			else
				mBlocks[MOTOR_2].cruiseInterval -= 32768;    // interval - whole number
			break;		
	}   // end switch()
}

 

//###############################################################
// runMotors() - The main attraction!!!
//
// Positions are set using setPosition().
//
// Motor driver should be enabled via 'setEnable()' prior to
// calling here.
//
// 'mSync' if true, synchronizes motor movement durations. This 
// happens even if using acceleration profile.
//
//###############################################################
//
uint8_t jStepper::runMotors(bool mSync)
{
	bool swap;
	uint32_t i;
	uint32_t startT;
	uint32_t endT;
	uint32_t rampSteps;
	uint32_t sTot = 0;
	uint32_t tTot;
	bool nothingToDo = true;
	float fr;
	union {
	   uint16_t val16[2];
	   uint32_t val32;
	}pun;

	// iterate through all motors
	for (i = 0; i < NUM_MOTORS; i++)
	{
                // TODO: resolve relative and absolute positioning.
		//
		// calc number of steps & direction
		//
        fr = mBlocks[i].newPosition - mBlocks[i].curPosition;
		if (fr >= 0.0)
			setDirection(i, MOTOR_DIRECTION_IN);
		else
			setDirection(i, MOTOR_DIRECTION_OUT);
        
        fr = fabs(fr);
		mBlocks[i].numSteps = round(fr * mBlocks[i].stepsPerMM);
		mBlocks[i].cruiseSteps = mBlocks[i].numSteps; // initialize cruise steps
        mBlocks[i].curPosition = mBlocks[i].newPosition; // update cur position

		if (mBlocks[i].numSteps > 0)
		{
			mBlocks[i].mAction = STEP_CRUISE;
			nothingToDo = false;

			//
			// calc feedrate and duration
			//
			fr = roundf(mBlocks[i].speed * mBlocks[i].stepsPerMM);   // calc cruise rate in PPS
			mBlocks[i].cruiseRate = (fr > MAX_CRUISE_RATE) ? MAX_CRUISE_RATE : fr;
			mBlocks[i].cruiseInterval = (1.0 / mBlocks[i].cruiseRate) * ONE_MILLION;
			mBlocks[i].cruiseReload = mBlocks[i].cruiseInterval;
			mBlocks[i].startInterval = mBlocks[i].cruiseInterval;

			//
			// calculate duration of movement
			//
			mBlocks[i].totalTime = uint32_t(mBlocks[i].cruiseInterval) * uint32_t(mBlocks[i].numSteps);
		}
		else	// no movement on this motor
		{
			mBlocks[i].mAction = STEP_DONE;
			mBlocks[i].totalTime = 0;
			mBlocks[i].cruiseRate = 0;
			mBlocks[i].cruiseInterval = 0;
			mBlocks[i].cruiseReload = 0;
		}
	}
	// bounce if no movements
	if (nothingToDo)
		return true;    // tell caller nothing to do

	//***************************************************************
	// Prepare for motor synchronization.
	//
	// Find the motor with the longest movement (distance * speed) and 
	// adjust the feedrate of the other motors to match duration.
	//
	// Use bubble sort to organize longest duration first.
	//
	for (i = ISRA; i < NUM_MOTORS; i++)
	{
		_sort[i] = i; 	// default ISR assignment motor 0,1,2
	}

	do {
		swap = false;
		for (i = MOTOR_0; i < NUM_MOTORS - 1; i++) {
			if (mBlocks[_sort[i]].totalTime < mBlocks[_sort[i + 1]].totalTime)
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
	// modify speeds to match motor with longest duration
	//
	for (i = ISRA; i < NUM_MOTORS; i++)
	{
		if (mBlocks[_sort[i]].numSteps > 0)
		{
			//
			// motor needs acceleration profile if acceleration > 0 and
			// target cruise speed is fast enough to need it.
			//
			if(mBlocks[_sort[i]].acceleration > 0.0 && mBlocks[_sort[i]].cruiseRate >= MOTOR_START_SPEED)
			{
				// calc first step interval
				mBlocks[_sort[i]].startInterval = ONE_MILLION / sqrt(2.0 * mBlocks[_sort[i]].acceleration);
				NOTMORE(mBlocks[_sort[i]].startInterval, MAX_ACCEL_INTERVAL);  // accel upper bound

				// calc total duration including accel / decel slopes
				mBlocks[_sort[i]].totalTime = 0;
				mBlocks[_sort[i]].rampTime = 0;
				startT = uint32_t(mBlocks[_sort[i]].startInterval);
				endT = uint32_t(mBlocks[_sort[i]].cruiseInterval);
				sTot = 0;
				tTot = 0;
				// iterative loop to get sum of ramp intervals and
				// number of steps from start to cruise speed.
				// Brute force yes, but it works ;>)
				for(rampSteps=0; rampSteps<LOOKUP_TBL_SIZE; rampSteps++)
				{
					// sum ramp intervals
					//Serial.print("nextI=");
					//Serial.println(startT);
					CALC_ACCEL(pun.val32, mBlocks[_sort[i]].startInterval, rampSteps+1);
					startT = pun.val16[1];
					if(startT <= endT)
						break;
                  
					sTot += startT;
                     
					// stretch shorter duration motors to match longest
					// duration. cruise speed must be adjusted on the fly
					// while building the ramp. Couldn't find a better
					// solution...
					if(i != ISRA && mSync)  // only motors with shorter durations
					{
						// adjust cruise speed on the fly
						// get sum of cruise intervals
						tTot = uint32_t(mBlocks[_sort[i]].cruiseSteps) - ((rampSteps+1) * 2);   
						tTot *= (endT);
						// plus accumulated ramp time
						tTot += (sTot * 2);
						// get ratio of this duration to longest 
						fr = float(mBlocks[_sort[ISRA]].totalTime) / float(tTot);		
						// keep tweaking until...
 						endT *= fr;
					}
					// If movement is short it may never reach cruise speed
					// resulting in a triangular trajectory.
					if(((rampSteps+1) * 2) >= mBlocks[_sort[i]].cruiseSteps)
						break;
				}
				
				// might have new cruise speed in 'endT'
				mBlocks[_sort[i]].cruiseInterval = endT;
				mBlocks[_sort[i]].cruiseReload = endT;
				mBlocks[_sort[i]].cruiseRate = (1.0 / endT) * ONE_MILLION;

				// if new speed is slow and doesn't need accel, we
				// kill accel profile and go with constant speed profile
				if(mBlocks[_sort[i]].cruiseRate < MOTOR_START_SPEED)
				{
					mBlocks[_sort[i]].rampSteps = 0;
					fr = float(mBlocks[_sort[ISRA]].totalTime) /
							float(mBlocks[_sort[i]].numSteps);
					mBlocks[_sort[i]].cruiseInterval = round(fr);
					mBlocks[_sort[i]].cruiseReload = mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].startInterval = mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].totalTime = mBlocks[_sort[i]].numSteps * mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].cruiseRate = (1.0 / float(mBlocks[_sort[i]].cruiseInterval)) * ONE_MILLION;
				}
				else    // still have accel - deal with ramp & cruise steps
				{
					// handle odd numbered steps
					if((rampSteps << 1) > mBlocks[_sort[i]].cruiseSteps)
						rampSteps--;
					mBlocks[_sort[i]].rampSteps	= rampSteps;
					mBlocks[_sort[i]].cruiseSteps -= (rampSteps * 2);
					mBlocks[_sort[i]].rampTime = sTot;
					mBlocks[_sort[i]].totalTime = mBlocks[_sort[i]].rampTime * 2;	// accel + decel
					mBlocks[_sort[i]].totalTime += (uint32_t(mBlocks[_sort[i]].cruiseSteps) *
							uint32_t(mBlocks[_sort[i]].cruiseInterval));
				}
			}
			//
			// constant cruise speed here
			//
			else
			{
				mBlocks[_sort[i]].rampSteps = 0;
				// sync motor durations here
				if(mSync && (i != ISRA))
				{
					// calc ratio of longest duration to this duration
					fr = float(mBlocks[_sort[ISRA]].totalTime) /
							float(mBlocks[_sort[i]].numSteps);
					// apply ratio to match duration
					mBlocks[_sort[i]].cruiseInterval = round(fr);
					mBlocks[_sort[i]].cruiseReload = mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].startInterval = mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].totalTime = mBlocks[_sort[i]].numSteps * mBlocks[_sort[i]].cruiseInterval;
					mBlocks[_sort[i]].cruiseRate = (1.0 / float(mBlocks[_sort[i]].cruiseInterval)) * ONE_MILLION;
				}
			}

			// prepare profile state
			mBlocks[_sort[i]].mAction = (mBlocks[_sort[i]].rampSteps > 0) ? STEP_ACCEL : STEP_CRUISE;
			mBlocks[_sort[i]].rampStepCount = 0;
		}
	}
    
   	            
	//***************************************************************
	// kick off the timer
	//
	// set up timer 1 for 10usec step pulse gen
	//
	cli();
	TCCRA = 0x00; 		// stop the timer
	TCCRB = 0x00;  	
	TIMSK = 0x00; 

   if(mBlocks[MOTOR_0].numSteps > 0) 
   {
	   if(mBlocks[MOTOR_0].cruiseInterval > 32767)   
	   {
		   mBlocks[MOTOR_0].mAction = STEP_LONG;  // set mode for long step intervals
		   OCRA = ((mBlocks[MOTOR_0].cruiseInterval & 0x7FFF) << 1);  // interrupt delay in usec *2
	   }
	   else
		   OCRA = (mBlocks[MOTOR_0].startInterval << 1);    // interrupt delay in usec *2
	  
	   SBI(TIMSK, OCIEA);	// enable compare A interrupt	
	   SBI(TIFR, OCFA);   	// clear any pending interrupts	   
   }
   
   if(mBlocks[MOTOR_1].numSteps > 0) 
   {
	   if(mBlocks[MOTOR_1].cruiseInterval > 32767)   
	   {
		   mBlocks[MOTOR_1].mAction = STEP_LONG;  // set mode for long step intervals  
		   OCRB = ((mBlocks[MOTOR_1].cruiseInterval & 0x7FFF) << 1);  
	   }
	   else
		   OCRB = (mBlocks[MOTOR_1].startInterval << 1);    // interrupt delay in usec *2

	   SBI(TIMSK, OCIEB);	// enable compare A interrupt	
	   SBI(TIFR, OCFB);   	// clear any pending interrupts	     	
   }
   
   if(mBlocks[MOTOR_2].numSteps > 0) 
   {
	   if(mBlocks[MOTOR_2].cruiseInterval > 32767)   
	   {
		   mBlocks[MOTOR_2].mAction = STEP_LONG;  // special mode for long step intervals
		   OCRC = ((mBlocks[MOTOR_2].cruiseInterval & 0x7FFF) << 1);      
	   }
	   else   
		   OCRC = (mBlocks[MOTOR_2].startInterval << 1);    // interrupt delay in usec *2

	   SBI(TIMSK, OCIEC);	// enable compare A interrupt	
	   SBI(TIFR, OCFC);   	// clear any pending interrupts	   
   }   

   TCNT = 0x00;				// start the timer running
   TCCRB = 0x02; 
   sei();
   return false;
}



//***************************************************************
// used to build lookup tables
//
void jStepper::genLookupTable(void) {

//
// set up initial parameters
//
float F = ONE_MILLION;
float acc = 100000;
// this is how _c0 is usually initialized
//uint32_t _c0 = round(0.676 * sqrt(2.0 / acc) * F); 
// but for this function _c0 is set to the outer bound
uint32_t _c0 = 32768;
uint32_t cruiseInt = 50;
uint32_t _cn;
float _magic = 65535;
uint32_t i;

   _cn = _c0;

	for(i=1; i<=1000; i++) 
	{
      Serial.print(uint32_t(_magic));
		Serial.print(", ");
		if(i % 10 == 0)
			Serial.println("");

      _cn = _cn - ((2.0 * _cn) / ((4.0 * i) + 1)); 
      _cn *=  float(1.0 - (i / 82000.0));     // build S curve at the end of the ramp
      
      _magic = float(_cn) / float(_c0);
      _magic = uint32_t(65536.0 * _magic);

      if(_cn < cruiseInt)
         break;
	}
}


#if defined(USE_TIMER1)
//***************************************************************
// interrupt handler for timer. 
//
ISR(TIMER1_COMPA_vect)
{
	psPtr->timerISRA();  // call class method to handle stepping
}

//***************************************************************
// 
ISR(TIMER1_COMPB_vect)
{
   psPtr->timerISRB();
}

//***************************************************************
// 
ISR(TIMER1_COMPC_vect)        // interrupt service routine
{
   psPtr->timerISRC();
}

#elif defined(USE_TIMER3)
//***************************************************************
// interrupt handler for timer. 
//
ISR(TIMER3_COMPA_vect)
{
	psPtr->timerISRA();  // call class method to handle stepping
}

//***************************************************************
// 
ISR(TIMER3_COMPB_vect)
{
   psPtr->timerISRB();
}

//***************************************************************
// 
ISR(TIMER3_COMPC_vect)        // interrupt service routine
{
   psPtr->timerISRC();
}

#elif defined(USE_TIMER4)
//***************************************************************
// interrupt handler for timer. 
//
ISR(TIMER4_COMPA_vect)
{
	psPtr->timerISRA();  // call class method to handle stepping
}

//***************************************************************
// 
ISR(TIMER4_COMPB_vect)
{
   psPtr->timerISRB();
}

//***************************************************************
// 
ISR(TIMER4_COMPC_vect)        // interrupt service routine
{
   psPtr->timerISRC();
}

#elif defined(USE_TIMER5)
//***************************************************************
// interrupt handler for timer. 
//
ISR(TIMER5_COMPA_vect)
{
	psPtr->timerISRA();  // call class method to handle stepping
}

//***************************************************************
// 
ISR(TIMER5_COMPB_vect)
{
   psPtr->timerISRB();
}

//***************************************************************
// 
ISR(TIMER5_COMPC_vect)        // interrupt service routine
{
   psPtr->timerISRC();
}
#endif
