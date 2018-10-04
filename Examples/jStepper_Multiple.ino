#include <jStepper.h>

// jStepper_Multiple Example demonstrates multiple instances of the library.
//
// jStepper use:
// 1) Download ZIP library and place in your Arduino 'libraries' directory
// 2) Install library using the Add .ZIP file in Arduino library manager.
// 3) In your sketch, create an instance of the library.
// 3) Create a jsMotorConfig object.
// 4) Initialize the jsMotorConfig object.
// 5) Initialize any other motor signals (external to library)
// 6) Set parameters for speed & distance.
// 7) Run the motors.
// 

// Two instances of the jStepper library
jStepper jstep0;
jStepper jstep1;

// Two motor template objects
jsMotorConfig mGroup0;
jsMotorConfig mGroup1;

void setup() 
{
  uint32_t k;
  Serial.begin(115200);
  
  // initialize I/O and other parameters in template
  initMotorTemplate(0);   // set up first group parameters
  initMotorTemplate(1);   // then second group
  
  // other specific motor initialization
  initMyMotors();   
    
  // send the template to the jStepper library 
  jstep0.begin(mGroup0);  
  jstep1.begin(mGroup1);  

  // Now ready to accept commands ---
  // set up basic prerequisites for one motor
  // set max travel distance in mm
  jstep0.setMaxPosition(200, 200, 80);
  jstep1.setMaxPosition(250, 0, 0);
    
  // set motor speed + limit
  jstep0.setMaxSpeed(100, 100, 100);
  jstep0.setSpeed(20, 20, 20);   // mm/sec (10mm/sec)

  jstep1.setMaxSpeed(100, 0, 0);
  jstep1.setSpeed(20, 20, 20);   // mm/sec (10mm/sec)  
  
  jstep0.setAcceleration(2500, 2500, 2500);
  jstep1.setAcceleration(2500, 2500, 2500);

  // set position known = true
  // this would normally be done after homing where the 
  // travel boundaries have been detected.
  jstep0.setPositionKnown(MOTOR_ALL, true);   // enable all motors in this group
  jstep1.setPositionKnown(MOTOR_ALL, true);           // enable first motor in this group  

  // enable the motor(s)
  jstep0.setEnabled(MOTOR_ALL, true);
  jstep1.setEnabled(MOTOR_ALL, true);  
}

void loop() {
  static bool someEvent = true;
  int8_t errcode;

  // here we do a simple movement
  if(someEvent)
  {
       
    // this will execute the plan and run the motor(s)
    // runMotors() is non-blocking and will return immediately
    //
    jstep0.runMotors(32.66, 21, 40, false, true);  // no sync, call planner
    jstep1.runMotors(86.23, 0, 0, false, true);
    
    // *** now wait for completion of move ***
    // NOTE: The step engine is interrupt driven and does
    // not need your program to wait. You can check for completion
    // whenever you want.
    while (jstep0.isRunning(MOTOR_ALL) || jstep1.isRunning(MOTOR_ALL));

    // subsequent moves
    //
    jstep0.runMotors(0, 0, 0, false, true);  
    jstep1.runMotors(0, 0, 0, false, true); 
    while (jstep0.isRunning(MOTOR_ALL) || jstep1.isRunning(MOTOR_ALL));

    delay(10);
    // disable the motor(s)
    jstep0.setEnabled(MOTOR_ALL, false);
    jstep1.setEnabled(MOTOR_ALL, false);  
    
    someEvent = false;    // just do it once
  }
}


//******************************************************
// General settings for 3 motors
// This structure must be sent to the library before
// anything else can happen.
//
void initMotorTemplate(uint8_t groupNum)
{
  switch(groupNum)
  {
    case 0:
      //--------------------------------
      // Timer must be selected here.
      // avoid using timers that are used 
      // in other libs and functions.
      //
      mGroup0.TIMER_SELECT = 1;       // use one of the 16 bit timers: 1, 3, 4, 5

      //--------------------------------
      // Define all motor I/O pins
      // Use 86 as pin number for unused pins
      //
      // Uses Arduino pin mapping
      //
      mGroup0.MOTOR_0_STEP_PIN = 37;
      mGroup0.MOTOR_0_DIR_PIN = 48;
      mGroup0.MOTOR_0_ENB_PIN = 29;
    
      mGroup0.MOTOR_1_STEP_PIN = 36;
      mGroup0.MOTOR_1_DIR_PIN = 49;
      mGroup0.MOTOR_1_ENB_PIN = 28;
    
      mGroup0.MOTOR_2_STEP_PIN = 35;
      mGroup0.MOTOR_2_DIR_PIN = 47;
      mGroup0.MOTOR_2_ENB_PIN = 27;
    
      mGroup0.ENDSTOP_MIN_0_PIN = 12;
      mGroup0.ENDSTOP_MIN_1_PIN = 11;
      mGroup0.ENDSTOP_MIN_2_PIN = 10;
    
      mGroup0.ENDSTOP_MAX_0_PIN = 86;     // unused pins
      mGroup0.ENDSTOP_MAX_1_PIN = 86;
      mGroup0.ENDSTOP_MAX_2_PIN = 86;

      // define logic states for various signals
      // 0 = LOW, 1 = HIGH
      //
      mGroup0.STEP_PULSE_ASSERT = 1;      // high active 
      mGroup0.MOTOR_ENABLE_LEVEL = 0;     // low enable active

      //--------------------------------
      // direction IN means moving away 
      // from the origin (not necessarily 
      // home position)
      //
      mGroup0.MOTOR_0_DIRECTION_IN = 0;   // each motor might define in/out differently
      mGroup0.MOTOR_1_DIRECTION_IN = 0;
      mGroup0.MOTOR_2_DIRECTION_IN = 0;

      // homing direction
      // set true if home position is != origin
      mGroup0.MOTOR_0_HOMING_INVERT = false;
      mGroup0.MOTOR_1_HOMING_INVERT = false;
      mGroup0.MOTOR_2_HOMING_INVERT = false;

      //--------------------------------
      // define endstop active state
      //
      mGroup0.IN_ENDSTOP = 1;             // endstop detector goes to a 1 when hit

      //--------------------------------
      // Basic geometry
      // motor steps per millimeter
      // Example: 
      // 1.8 deg motor = 200 steps/rev
      // with 1/4 microstep = 800 steps/rev
      // Leadscrew moves carriage 8mm/rev
      // Steps/mm = 800 / 8 = 100
      //
      mGroup0.MOTOR_0_STEPS_PER_MM = 100;
      mGroup0.MOTOR_1_STEPS_PER_MM = 100;
      mGroup0.MOTOR_2_STEPS_PER_MM = 100;
      break;

    case 1:  // second motor group
      mGroup1.TIMER_SELECT = 3;       // can use one of the 16 bit timers: 1, 3, 4, 5

      mGroup1.MOTOR_0_STEP_PIN = 34;
      mGroup1.MOTOR_0_DIR_PIN = 43;
      mGroup1.MOTOR_0_ENB_PIN = 26;
    
      mGroup1.MOTOR_1_STEP_PIN = 86;
      mGroup1.MOTOR_1_DIR_PIN = 86;
      mGroup1.MOTOR_1_ENB_PIN = 86;
    
      mGroup1.MOTOR_2_STEP_PIN = 86;
      mGroup1.MOTOR_2_DIR_PIN = 86;
      mGroup1.MOTOR_2_ENB_PIN = 86;
    
      mGroup1. ENDSTOP_MIN_0_PIN = 12;
      mGroup1.ENDSTOP_MIN_1_PIN = 86;
      mGroup1.ENDSTOP_MIN_2_PIN = 86;
    
      mGroup1.ENDSTOP_MAX_0_PIN = 86;     // unused pins
      mGroup1.ENDSTOP_MAX_1_PIN = 86;
      mGroup1.ENDSTOP_MAX_2_PIN = 86;
    
      mGroup1.STEP_PULSE_ASSERT = 1;      // high active 
      mGroup1.MOTOR_ENABLE_LEVEL = 0;     // low enable active

      mGroup1.MOTOR_0_DIRECTION_IN = 0;   // each motor might define in/out differently
      mGroup1.MOTOR_1_DIRECTION_IN = 0;
      mGroup1.MOTOR_2_DIRECTION_IN = 0;

      mGroup1.MOTOR_0_HOMING_INVERT = false;
      mGroup1.MOTOR_1_HOMING_INVERT = false;
      mGroup1.MOTOR_2_HOMING_INVERT = false;
      

      mGroup1.IN_ENDSTOP = 1;             // endstop detector goes to a 1 when hit

      mGroup1.MOTOR_0_STEPS_PER_MM = 100;
      mGroup1.MOTOR_1_STEPS_PER_MM = 100;
      mGroup1.MOTOR_2_STEPS_PER_MM = 100;
      break;            
  }
}


//******************************************************
// initialize motor control signals specific to the
// hardware and driver chip. This hardware has four A4982
// driver chips.
//
void initMyMotors(void)
{
#define M0_MS1_PIN          40
#define M0_MS2_PIN          41

#define M1_MS1_PIN          69
#define M1_MS2_PIN          39

#define M2_MS1_PIN          68
#define M2_MS2_PIN          67

#define M3_MS1_PIN          65
#define M3_MS2_PIN          66

#define M01_CURRENT_PIN     46    // motor 0 & 1 share same current setting
#define M2_CURRENT_PIN      45
#define M3_CURRENT_PIN      44

// set microstepping for 1/4
  pinMode(M0_MS1_PIN, OUTPUT);  // mtr 0 MS1 microstep select
  digitalWrite(M0_MS1_PIN, LOW);
  pinMode(M0_MS2_PIN, OUTPUT);
  digitalWrite(M0_MS2_PIN, HIGH);

  pinMode(M1_MS1_PIN, OUTPUT);  // mtr 0 MS1 microstep select
  digitalWrite(M1_MS1_PIN, LOW);
  pinMode(M1_MS2_PIN, OUTPUT);
  digitalWrite(M1_MS2_PIN, HIGH);  

  pinMode(M2_MS1_PIN, OUTPUT);  // mtr 0 MS1 microstep select
  digitalWrite(M2_MS1_PIN, LOW);
  pinMode(M2_MS2_PIN, OUTPUT);
  digitalWrite(M2_MS2_PIN, HIGH);  

  pinMode(M3_MS1_PIN, OUTPUT);  // mtr 0 MS1 microstep select
  digitalWrite(M3_MS1_PIN, LOW);
  pinMode(M3_MS2_PIN, OUTPUT);
  digitalWrite(M3_MS2_PIN, HIGH);      

  // set motor current - this hardware uses a filtered PWM to
  // develop an analog reference for the driver chip
  pinMode(M01_CURRENT_PIN, OUTPUT);
  analogWrite(M01_CURRENT_PIN, 128);   // about 1A

  pinMode(M2_CURRENT_PIN, OUTPUT);
  analogWrite(M2_CURRENT_PIN, 128);

  pinMode(M3_CURRENT_PIN, OUTPUT);
  analogWrite(M3_CURRENT_PIN, 128);    
}
