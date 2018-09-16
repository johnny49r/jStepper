#include <jStepper.h>
//
// jStepper use:
// 1) Download library and place in your Arduino 'libraries' directory
// 2) Install library using the Add .ZIP file in Arduino library manager.
// 3) Edit the jsconfig.h file in the jStepper library folder to
//    specify pin assignments and geometry.
//
// This example 

jStepper jstep;

void setup() 
{
  Serial.begin(115200);
  initMyMotor();  // init unique motor control signals
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
    // set a new position for the first motor in millimeters
    jstep.setPosition(32.66, 0, 0);
    jstep.setPositionKnown(0, true);
    // this will execute the plan and run the motor(s)
    errcode = jstep.runMotors(false);
    // *** now wait for completion of move ***
    // NOTE: The step engine is interrupt driven and does
    // not need your program to wait. You can check for completion
    // whenever you want.
    while (jstep.isRunning(MOTOR_ALL));

    someEvent = false;
  }
}

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
