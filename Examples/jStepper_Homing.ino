#include <jStepper.h>

//
// jstep use:
// 1) Download library and place in your Arduino 'libraries' directory
// 2) Install library using the Add .ZIP file in Arduino library manager.
// 3) Edit the jsconfig.h file in the jstep library folder to
//    specify pin assignments and geometry.
//

// New instance of the jStepper library
jStepper jstep;

// create a motor template object
jsMotorConfig mGroup0;


void setup() 
{
  Serial.begin(115200);
  initMotorTemplate(0);   // only one group in this example
  initMyMotor();  // init unique motor control signals
  jstep.begin(mGroup0); 
  // set up basic prerequisites for one motor
  // set max travel limits in mm
  jstep.setMaxPosition(200, 0, 0);
  // set motor speed
  jstep.setMaxSpeed(100, 0, 0);
  jstep.setSpeed(30, 0, 0);   // mm/sec (10mm/sec)
  jstep.setEnabled(MOTOR_0, true);
}

void loop() {
  static bool someEvent = true;
  int8_t errcode;

  if(someEvent)
  {
    // home the first motor and set the PositionKnown true if successful.
    jstep.homeMotor(MOTOR_0, 500);
    while (jstep.isRunning(MOTOR_ALL));

    // final result of homing not known until homing is complete
    errcode = jstep.getLastResult(MOTOR_0);
    if(errcode == ERR_NONE)
      jstep.setPositionKnown(MOTOR_0, true);
    
    // this will plan the movement and run the motor(s)
    errcode = jstep.runMotors(38.34, 0, 0, false, true);
    
    // *** now wait for completion of move ***
    // NOTE: The step engine is interrupt driven (non-blocking) and does
    // not need your program to wait. You can check for completion
    // whenever you want.
    // 
    // NOTE: If you issue another 'runMotors' before the first 
    // operation has completed the function will return ERR_MOTORS_RUNNING.
    //
    while (jstep.isRunning(MOTOR_ALL));

    // return motor to start position
    errcode = jstep.runMotors(0, 0, 0, false, true);
    while (jstep.isRunning(MOTOR_ALL));    // wait for completion

    delay(100);
    
    // all done for now - disable the motor
    jstep.setEnabled(MOTOR_0, false);
    someEvent = false;  // run one time only
  }
}

//**********************************************************
// initialize motor control signals specific 
// to your driver chip. This driver is an A4982
void initMyMotor(void)
{
#define M0_MS1_PIN          40
#define M0_MS2_PIN          41
#define M0_CURRENT_PIN      46

// set microstepping for 1/4
  pinMode(M0_MS1_PIN, OUTPUT);  // mtr 0 MS1 microstep select
  digitalWrite(M0_MS1_PIN, LOW);
  pinMode(M0_MS2_PIN, OUTPUT);
  digitalWrite(M0_MS2_PIN, HIGH);

  // set motor current - this hardware uses a filtered PWM to
  // develop an analog reference for the driver chip
  pinMode(M0_CURRENT_PIN, OUTPUT);
  analogWrite(M0_CURRENT_PIN, 128);   // about 1A
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
    
      mGroup0. ENDSTOP_MIN_0_PIN = 12;
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
  }
}
